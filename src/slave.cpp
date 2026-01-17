#include "../include/slave.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "../include/common.h"
#include "../include/messages.h"

#include "driver/ledc.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "../include/rgb_led.h"
#include "esp_log.h"
#include "esp_random.h"

static const char *TAG = "slave";

static ParamsPayload s_params = {
    .speed = 0.0f, .probability = 0.0f, .mode = 0};
static uint32_t s_last_rx_ms = 0;
static uint32_t s_last_update_ms = 0;
static uint32_t s_last_prob_ms = 0;
static uint32_t s_last_hello_ms = 0;
static uint32_t s_seq = 0;
static uint8_t s_hello_burst_remaining = 0;
static uint32_t s_last_hello_burst_ms = 0;
static uint32_t s_last_rejoin_burst_ms = 0;
static uint32_t s_next_hello_ms = 0;
static uint32_t s_next_burst_ms = 0;
static bool s_prob_on = false;
static uint8_t s_prev_mode = 0;
static float s_current_speed = 0.0f;
static float s_target_speed = 0.0f;
static bool s_link_lost = false;
static bool s_led_on = false;
static RgbLed s_led = {};
static bool s_flash_active = false;
static bool s_flash_on = false;
static uint8_t s_flash_remaining = 0;
static uint32_t s_flash_next_ms = 0;
static bool s_params_dirty = false;
static uint32_t s_last_params_save_ms = 0;

static float clampFloat(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

static float clampSpeed(float speed) {
  return clampFloat(speed, SPEED_MIN, SPEED_MAX);
}

static float clampProbability(float prob) {
  return clampFloat(prob, PROBABILITY_MIN, PROBABILITY_MAX);
}

static float approachFloat(float current, float target, float maxStep) {
  if (current < target) {
    float next = current + maxStep;
    return (next > target) ? target : next;
  }
  if (current > target) {
    float next = current - maxStep;
    return (next < target) ? target : next;
  }
  return current;
}

static void setPwmSpeed(float speed) {
  float clamped = clampSpeed(speed);
  uint32_t max_duty = (1u << PWM_RES_BITS) - 1u;
  uint32_t duty = (uint32_t)lroundf(clamped * (float)max_duty);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL);
}

static void setLed(bool on) {
  if (on) {
    rgbLedSet(&s_led, LED_WHITE_LEVEL, LED_WHITE_LEVEL, LED_WHITE_LEVEL);
  } else {
    rgbLedSet(&s_led, 0, 0, 0);
  }
  s_led_on = on;
}

static void setWhiteBySpeed(float speed) {
  float clamped = clampSpeed(speed);
  uint8_t level = (uint8_t)lroundf(clamped * (float)LED_WHITE_LEVEL);
  rgbLedSet(&s_led, level, level, level);
  s_led_on = (level != 0);
}

static void setGreen(bool on) {
  if (on) {
    rgbLedSet(&s_led, 0, LED_GREEN_LEVEL, 0);
  } else {
    rgbLedSet(&s_led, 0, 0, 0);
  }
  s_led_on = on;
}

static void startParamFlash(uint32_t now) {
  s_flash_active = true;
  s_flash_on = false;
  s_flash_remaining = (uint8_t)(LED_PARAM_FLASH_COUNT * 2);
  s_flash_next_ms = now;
}

static void saveParamsToNvs() {
  nvs_handle_t handle = 0;
  esp_err_t ret = nvs_open("leaf", NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "nvs open failed err=%d", (int)ret);
    return;
  }
  ret = nvs_set_blob(handle, "params", &s_params, sizeof(s_params));
  if (ret == ESP_OK) {
    ret = nvs_commit(handle);
  }
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "nvs save failed err=%d", (int)ret);
  }
  nvs_close(handle);
}

static void loadParamsFromNvs() {
  nvs_handle_t handle = 0;
  esp_err_t ret = nvs_open("leaf", NVS_READONLY, &handle);
  if (ret != ESP_OK) {
    return;
  }
  ParamsPayload stored = {};
  size_t len = sizeof(stored);
  ret = nvs_get_blob(handle, "params", &stored, &len);
  nvs_close(handle);
  if (ret == ESP_OK && len == sizeof(stored)) {
    s_params.speed = clampSpeed(stored.speed);
    s_params.probability = clampProbability(stored.probability);
    s_params.mode = (stored.mode != 0) ? 1 : 0;
    ESP_LOGI(TAG, "restored params speed=%.3f prob=%.3f mode=%u",
             (double)s_params.speed, (double)s_params.probability,
             (unsigned)s_params.mode);
  }
}

static bool paramsChanged(const ParamsPayload *next) {
  if (next == nullptr) {
    return false;
  }
  if (next->speed != s_params.speed) {
    return true;
  }
  if (next->probability != s_params.probability) {
    return true;
  }
  if (next->mode != s_params.mode) {
    return true;
  }
  return false;
}

static void updateProbabilityDecision(uint32_t now) {
  if (s_params.probability <= 0.0f) {
    s_prob_on = false;
  } else if (s_params.probability >= 1.0f) {
    s_prob_on = true;
  } else {
    float r = (float)esp_random() / (float)UINT32_MAX;
    s_prob_on = (r < s_params.probability);
  }
  s_last_prob_ms = now;
}

static void handlePacket(const MeshPacket *pkt) {
  if (pkt->type == MSG_TYPE_PARAMS) {
    bool changed = paramsChanged(&pkt->params);
    s_params.speed = clampSpeed(pkt->params.speed);
    s_params.probability = clampProbability(pkt->params.probability);
    s_params.mode = (pkt->params.mode != 0) ? 1 : 0;
    s_last_rx_ms = millis();
    s_link_lost = false;
    ESP_LOGI(TAG, "params speed=%.3f prob=%.3f mode=%u",
             (double)s_params.speed, (double)s_params.probability,
             (unsigned)s_params.mode);
    if (changed) {
      startParamFlash(s_last_rx_ms);
      s_params_dirty = true;
    }
  }
}

static void relayPacketIfNeeded(const MeshPacket *pkt) {
  if (pkt->type == MSG_TYPE_ACK || pkt->type == MSG_TYPE_DISCOVER) {
    return;
  }
  if (pkt->ttl == 0) {
    return;
  }
  MeshPacket relay = *pkt;
  relay.ttl--;
  esp_err_t ret = radioSendBroadcast(&relay, sizeof(relay));
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "relay failed err=%d", (int)ret);
  }
}

static void sendHello() {
  MeshPacket pkt = {};
  pkt.version = MSG_VERSION;
  pkt.size = sizeof(MeshPacket);
  pkt.type = MSG_TYPE_HELLO;
  pkt.ttl = RELAY_TTL_DEFAULT;
  pkt.seq = ++s_seq;
  pkt.origin_id = getNodeId();

  esp_err_t ret = radioSendBroadcast(&pkt, sizeof(pkt));
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "hello send failed err=%d", (int)ret);
  }
}

static uint32_t jitterMs(uint32_t base, uint32_t jitter) {
  if (jitter == 0) {
    return base;
  }
  uint32_t span = jitter * 2 + 1;
  int32_t offset = (int32_t)(esp_random() % span) - (int32_t)jitter;
  int32_t out = (int32_t)base + offset;
  return (out < 0) ? 0u : (uint32_t)out;
}

static void receivedCallback(const esp_now_recv_info_t *recv_info,
                             const uint8_t *data, int len) {
  if (recv_info == nullptr) {
    ESP_LOGW(TAG, "rx with null info len=%d", len);
    return;
  }
  radioNotePeer(recv_info->src_addr);
  radioNoteRx();

  if (data == nullptr || len < (int)sizeof(MeshPacket)) {
    return;
  }

  MeshPacket pkt = {};
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.version != MSG_VERSION || pkt.size != sizeof(MeshPacket)) {
    return;
  }
  if (pkt.origin_id == getNodeId()) {
    return;
  }
  if (radioRememberPacket(pkt.origin_id, pkt.seq)) {
    return;
  }

  if (pkt.type == MSG_TYPE_DISCOVER) {
    MeshPacket ack = {};
    ack.version = MSG_VERSION;
    ack.size = sizeof(MeshPacket);
    ack.type = MSG_TYPE_ACK;
    ack.ttl = 0;
    ack.seq = ++s_seq;
    ack.origin_id = getNodeId();
    esp_err_t ret = radioSendUnicast(recv_info->src_addr, &ack, sizeof(ack));
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "ack send failed err=%d", (int)ret);
    }
    return;
  }

  handlePacket(&pkt);
  relayPacketIfNeeded(&pkt);
}

void setupSlave() {
  setupRadio();
  radioSetReceiveCallback(&receivedCallback);
  printNodeId("SLAVE");

  if (!rgbLedInit(&s_led, (gpio_num_t)LED_GPIO)) {
    ESP_LOGW(TAG, "rgb led init failed");
  }
  setLed(false);

  // Assumes PWM_GPIO is wired to the motor driver's speed input.
  ledc_timer_config_t timer_cfg = {};
  timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_cfg.timer_num = (ledc_timer_t)PWM_TIMER;
  timer_cfg.duty_resolution = (ledc_timer_bit_t)PWM_RES_BITS;
  timer_cfg.freq_hz = PWM_FREQ_HZ;
  timer_cfg.clk_cfg = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

  ledc_channel_config_t channel_cfg = {};
  channel_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
  channel_cfg.channel = (ledc_channel_t)PWM_CHANNEL;
  channel_cfg.timer_sel = (ledc_timer_t)PWM_TIMER;
  channel_cfg.intr_type = LEDC_INTR_DISABLE;
  channel_cfg.gpio_num = PWM_GPIO;
  channel_cfg.duty = 0;
  channel_cfg.hpoint = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));

  s_last_rx_ms = millis();
  s_last_update_ms = s_last_rx_ms;
  s_last_prob_ms = s_last_rx_ms;
  s_last_hello_ms = s_last_rx_ms;
  s_last_hello_burst_ms = s_last_rx_ms;
  s_last_rejoin_burst_ms = s_last_rx_ms;
  s_hello_burst_remaining = SLAVE_HELLO_BURST_COUNT;
  s_next_hello_ms =
      s_last_rx_ms +
      jitterMs(SLAVE_HELLO_INTERVAL_MS, SLAVE_HELLO_JITTER_MS);
  s_next_burst_ms =
      s_last_rx_ms +
      jitterMs(SLAVE_HELLO_BURST_INTERVAL_MS, SLAVE_BURST_JITTER_MS);
  s_prev_mode = s_params.mode;

  loadParamsFromNvs();
  setWhiteBySpeed(s_params.speed);

  // Kick an immediate hello so masters discover quickly on boot.
  sendHello();
}

void loopSlave() {
  uint32_t now = millis();

  if ((now - s_last_rx_ms) > RX_TIMEOUT_MS) {
    if (!s_link_lost) {
      ESP_LOGW(TAG, "rx timeout, stopping motor");
      s_link_lost = true;
    }
  }

  if (s_params.mode != s_prev_mode) {
    s_prev_mode = s_params.mode;
    s_last_prob_ms = 0;
    if (s_params.mode == 0) {
      s_prob_on = true;
    } else {
      updateProbabilityDecision(now);
    }
  }

  if (s_params.mode != 0) {
    if ((now - s_last_prob_ms) >= PROBABILITY_UPDATE_INTERVAL_MS) {
      updateProbabilityDecision(now);
    }
  } else {
    s_prob_on = true;
  }

  if (s_link_lost) {
    s_target_speed = 0.0f;
    if ((now - s_last_rejoin_burst_ms) >= SLAVE_REJOIN_BURST_INTERVAL_MS) {
      s_hello_burst_remaining = SLAVE_HELLO_BURST_COUNT;
      s_last_rejoin_burst_ms = now;
      s_next_burst_ms =
          now +
          jitterMs(SLAVE_HELLO_BURST_INTERVAL_MS, SLAVE_BURST_JITTER_MS);
    }
  } else if (s_params.mode == 0) {
    s_target_speed = s_params.speed;
  } else {
    s_target_speed = s_prob_on ? s_params.speed : 0.0f;
  }

  uint32_t dt_ms = now - s_last_update_ms;
  s_last_update_ms = now;
  float dt_s = (float)dt_ms / 1000.0f;
  float max_step = SPEED_SLEW_PER_SEC * dt_s;
  s_current_speed = approachFloat(s_current_speed, s_target_speed, max_step);
  setPwmSpeed(s_current_speed);

  if (s_link_lost) {
    s_flash_active = false;
    setLed(false);
  } else if (s_flash_active) {
    if (now >= s_flash_next_ms) {
      s_flash_on = !s_flash_on;
      setGreen(s_flash_on);
      if (s_flash_remaining > 0) {
        s_flash_remaining--;
      }
      s_flash_next_ms = now + LED_PARAM_FLASH_INTERVAL_MS;
      if (s_flash_remaining == 0) {
        s_flash_active = false;
        setWhiteBySpeed(s_params.speed);
      }
    }
  } else {
    // Follow effective target so probabilistic mode reflects on/off behavior.
    setWhiteBySpeed(s_target_speed);
  }

  if (s_params_dirty &&
      (now - s_last_params_save_ms) >= PARAMS_SAVE_DEBOUNCE_MS) {
    saveParamsToNvs();
    s_params_dirty = false;
    s_last_params_save_ms = now;
  }

  if (now >= s_next_hello_ms) {
    sendHello();
    s_last_hello_ms = now;
    s_next_hello_ms =
        now + jitterMs(SLAVE_HELLO_INTERVAL_MS, SLAVE_HELLO_JITTER_MS);
  }

  if (s_hello_burst_remaining > 0 && now >= s_next_burst_ms) {
    sendHello();
    s_last_hello_burst_ms = now;
    s_next_burst_ms =
        now +
        jitterMs(SLAVE_HELLO_BURST_INTERVAL_MS, SLAVE_BURST_JITTER_MS);
    s_hello_burst_remaining--;
  }
}
