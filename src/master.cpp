#include "../include/master.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "../include/common.h"
#include "../include/messages.h"

#include "../include/rgb_led.h"
#include "driver/uart.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_random.h"

static const char *TAG = "master";

static uint32_t s_seq = 0;
static uint32_t s_last_send_ms = 0;
static uint32_t s_last_hello_ms = 0;
static uint32_t s_last_status_ms = 0;
static uint32_t s_last_peer_count = 0;
static uint32_t s_last_peer_check_ms = 0;
static uint32_t s_next_discover_ms = 0;
static uint32_t s_last_led_ms = 0;
static bool s_led_on = false;
static RgbLed s_led = {};
static bool s_params_dirty = false;
static uint32_t s_last_params_save_ms = 0;
static uint32_t s_led_flash_until_ms = 0;
static bool s_force_send = false;
static uint32_t s_last_mac_log_ms = 0;

/**
 * mode = 0 → Direct/continuous: 
 *            the slave always runs at speed 
 *            (probability ignored).
 * mode = 1 → Probabilistic: 
 *            the slave periodically decides 
 *            to run or stop based on probability. 
 *            If the random draw is “off,” speed 
 *            becomes 0 until the next decision 
 *            interval.
 */
static ParamsPayload s_params = {
    .speed = 0.0f, .probability = 1.0f, .mode = 0};
static ParamsPayload s_last_sent = {
    .speed = -1.0f, .probability = -1.0f, .mode = 0};

static char s_line_buf[128] = {0};
static size_t s_line_len = 0;

static float clampFloat(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

static float normalizeSpeed(float speed) {
  // GUI sends 0.0 .. 1.0
  return clampFloat(speed, SPEED_MIN, SPEED_MAX);
}

static float normalizeProbability(float prob) {
  // GUI sends 0.0 .. 1.0
  return clampFloat(prob, PROBABILITY_MIN, PROBABILITY_MAX);
}

static void triggerFlash(uint32_t now);

static void sendPacket(uint8_t type, const ParamsPayload *params) {
  MeshPacket pkt = {};
  pkt.version = MSG_VERSION;
  pkt.size = sizeof(MeshPacket);
  pkt.type = type;
  pkt.ttl = RELAY_TTL_DEFAULT;
  pkt.seq = ++s_seq;
  pkt.origin_id = getNodeId();
  if (params != nullptr) {
    pkt.params = *params;
  }

  esp_err_t send_ret = radioSendBroadcast(&pkt, sizeof(pkt));
  if (send_ret != ESP_OK) {
    ESP_LOGW(TAG, "send failed err=%d", (int)send_ret);
  } else {
    triggerFlash(millis());
  }
}

static void sendDiscover() {
  MeshPacket pkt = {};
  pkt.version = MSG_VERSION;
  pkt.size = sizeof(MeshPacket);
  pkt.type = MSG_TYPE_DISCOVER;
  pkt.ttl = 0;
  pkt.seq = ++s_seq;
  pkt.origin_id = getNodeId();

  esp_err_t send_ret = radioSendBroadcast(&pkt, sizeof(pkt));
  if (send_ret != ESP_OK) {
    ESP_LOGW(TAG, "discover send failed err=%d", (int)send_ret);
  } else {
    triggerFlash(millis());
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

static bool parseKeyValue(const char *token, ParamsPayload *params) {
  const char *eq = strchr(token, '=');
  if (eq == nullptr) {
    return false;
  }

  size_t key_len = (size_t)(eq - token);
  if (key_len == 0) {
    return false;
  }

  char key[16] = {0};
  size_t copy_len = key_len < (sizeof(key) - 1) ? key_len : (sizeof(key) - 1);
  for (size_t i = 0; i < copy_len; ++i) {
    key[i] = (char)tolower((unsigned char)token[i]);
  }

  const char *val_str = eq + 1;
  if (strcmp(key, "speed") == 0) {
    params->speed = normalizeSpeed(strtof(val_str, nullptr));
    return true;
  }
  if (strcmp(key, "prob") == 0 || strcmp(key, "probability") == 0) {
    params->probability = normalizeProbability(strtof(val_str, nullptr));
    return true;
  }
  if (strcmp(key, "mode") == 0) {
    int mode = (int)strtol(val_str, nullptr, 10);
    params->mode = (mode != 0) ? 1 : 0;
    return true;
  }

  return false;
}

static bool parseLine(const char *line, ParamsPayload *params) {
  if (line == nullptr || params == nullptr) {
    return false;
  }

  char buf[128] = {0};
  strncpy(buf, line, sizeof(buf) - 1);

  bool updated = false;
  bool has_kv = (strchr(buf, '=') != nullptr);

  if (has_kv) {
    char *save = nullptr;
    for (char *tok = strtok_r(buf, ", \t", &save); tok != nullptr;
         tok = strtok_r(nullptr, ", \t", &save)) {
      if (parseKeyValue(tok, params)) {
        updated = true;
      }
    }
    return updated;
  }

  char *save = nullptr;
  char *tok = strtok_r(buf, ", \t", &save);
  if (tok == nullptr) {
    return false;
  }
  params->speed = normalizeSpeed(strtof(tok, nullptr));

  tok = strtok_r(nullptr, ", \t", &save);
  if (tok == nullptr) {
    return false;
  }
  params->probability = normalizeProbability(strtof(tok, nullptr));

  tok = strtok_r(nullptr, ", \t", &save);
  if (tok == nullptr) {
    return false;
  }
  params->mode = (strtol(tok, nullptr, 10) != 0) ? 1 : 0;
  return true;
}

static void handleSerial() {
  
  uint8_t rx_buf[64];
  int len = uart_read_bytes(UART_NUM_0, rx_buf, sizeof(rx_buf), 0);

  if (len <= 0) {
    return;
  }

  for (int i = 0; i < len; ++i) {
    char c = (char)rx_buf[i];
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      s_line_buf[s_line_len] = '\0';
      if (s_line_len > 0) {
        ParamsPayload new_params = s_params;
        if (parseLine(s_line_buf, &new_params)) {
          s_params = new_params;
          ESP_LOGI(TAG, "params speed=%.3f prob=%.3f mode=%u",
                   (double)s_params.speed, (double)s_params.probability,
                   (unsigned)s_params.mode);
          s_params_dirty = true;
          s_force_send = true;
          char ack_buf[96];
          int ack_len = snprintf(
              ack_buf, sizeof(ack_buf), "ACK speed=%.3f prob=%.3f mode=%u\r\n",
              (double)s_params.speed, (double)s_params.probability,
              (unsigned)s_params.mode);
          if (ack_len > 0) {
            uart_write_bytes(UART_NUM_0, ack_buf, ack_len);
          }
        } else {
          ESP_LOGW(TAG, "invalid line: %s", s_line_buf);
        }
      }
      s_line_len = 0;
      continue;
    }

    if (s_line_len < (sizeof(s_line_buf) - 1)) {
      s_line_buf[s_line_len++] = c;
    }
  }
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

  triggerFlash(millis());

  if (pkt.type == MSG_TYPE_ACK) {
    return;
  }

  if (pkt.ttl > 0) {
    MeshPacket relay = pkt;
    relay.ttl--;
    esp_err_t ret = radioSendBroadcast(&relay, sizeof(relay));
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "relay failed err=%d", (int)ret);
    }
  }
}

static void setLed(bool on) {
  if (on) {
    rgbLedSet(&s_led, LED_RED_LEVEL, 0, 0);
  } else {
    rgbLedSet(&s_led, 0, 0, 0);
  }
  s_led_on = on;
}

static void setLedWhite(bool on) {
  if (on) {
    rgbLedSet(&s_led, LED_DATA_LEVEL, LED_DATA_LEVEL, LED_DATA_LEVEL);
  } else {
    rgbLedSet(&s_led, 0, 0, 0);
  }
}

static void triggerFlash(uint32_t now) {
  uint32_t until = now + MASTER_LED_FLASH_MS;
  if (until > s_led_flash_until_ms) {
    s_led_flash_until_ms = until;
  }
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
    s_params.speed = normalizeSpeed(stored.speed);
    s_params.probability = normalizeProbability(stored.probability);
    s_params.mode = (stored.mode != 0) ? 1 : 0;
    ESP_LOGI(TAG, "restored params speed=%.3f prob=%.3f mode=%u",
             (double)s_params.speed, (double)s_params.probability,
             (unsigned)s_params.mode);
  }
}

void setupMaster() {
  setupRadio();
  radioSetReceiveCallback(&receivedCallback);
  printNodeId("MASTER");

  if (!rgbLedInit(&s_led, (gpio_num_t)LED_GPIO)) {
    ESP_LOGW(TAG, "rgb led init failed");
  }
  setLed(false);

  loadParamsFromNvs();

  uart_config_t cfg = {};
  cfg.baud_rate = (int)SERIAL_BAUD_RATE;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, SERIAL_RX_BUF, SERIAL_TX_BUF, 0,
                                      nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &cfg));
}

void loopMaster() {
  uint32_t now = millis();

  handleSerial();

  bool changed = false;
  if (s_params.speed != s_last_sent.speed) {
    changed = true;
  }
  if (s_params.probability != s_last_sent.probability) {
    changed = true;
  }
  if (s_params.mode != s_last_sent.mode) {
    changed = true;
  }

  if (s_force_send ||
      changed || (now - s_last_send_ms) >= HEARTBEAT_INTERVAL_MS) {
    sendPacket(MSG_TYPE_PARAMS, &s_params);
    s_last_sent = s_params;
    s_last_send_ms = now;
    s_force_send = false;
  }

  if ((now - s_last_hello_ms) >= HELLO_INTERVAL_MS) {
    sendPacket(MSG_TYPE_HELLO, nullptr);
    s_last_hello_ms = now;
  }

  if (s_next_discover_ms == 0) {
    s_next_discover_ms =
        now + jitterMs(DISCOVER_INTERVAL_MS, DISCOVER_JITTER_MS);
  }
  if (now >= s_next_discover_ms) {
    sendDiscover();
    s_next_discover_ms =
        now + jitterMs(DISCOVER_INTERVAL_MS, DISCOVER_JITTER_MS);
  }

  if ((now - s_last_status_ms) >= 5000) {
    logNodeList();

    ESP_LOGI(TAG, "speed=%.3f", (double)s_params.speed);
    ESP_LOGI(TAG, "probability=%.3f", (double)s_params.probability);
    ESP_LOGI(TAG, "mode=%u", (unsigned)s_params.mode);
    RadioStats stats = {};
    radioGetStats(&stats);
    uint32_t tx_age = (stats.last_tx_ms == 0) ? 0 : (now - stats.last_tx_ms);
    uint32_t rx_age = (stats.last_rx_ms == 0) ? 0 : (now - stats.last_rx_ms);
    ESP_LOGI(TAG,
             "radio tx=%u ok=%u fail=%u rx=%u tx_age=%ums rx_age=%ums",
             (unsigned)stats.tx_attempts, (unsigned)stats.tx_success,
             (unsigned)stats.tx_fail, (unsigned)stats.rx_packets,
             (unsigned)tx_age, (unsigned)rx_age);

    s_last_status_ms = now;
  }

  if ((now - s_last_peer_check_ms) >= PEER_CHECK_INTERVAL_MS) {
    uint32_t peer_count = radioUpdatePeerCount();
    if (peer_count != s_last_peer_count) {
      ESP_LOGI(TAG, "slaves=%u", (unsigned)peer_count);
      s_last_peer_count = peer_count;
    }
    s_last_peer_check_ms = now;
  }

  if ((now - s_last_led_ms) >= LED_BLINK_INTERVAL_MS) {
    setLed(!s_led_on);
    s_last_led_ms = now;
  }

  if ((now - s_last_mac_log_ms) >= MAC_LOG_INTERVAL_MS) {
    uint8_t mac[6] = {0};
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
      ESP_LOGI(TAG, "mac=%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2],
               mac[3], mac[4], mac[5]);
    }
    s_last_mac_log_ms = now;
  }

  if (s_led_flash_until_ms != 0) {
    if (now < s_led_flash_until_ms) {
      setLedWhite(true);
    } else {
      s_led_flash_until_ms = 0;
      setLed(s_led_on);
    }
  }

  if (s_params_dirty &&
      (now - s_last_params_save_ms) >= PARAMS_SAVE_DEBOUNCE_MS) {
    saveParamsToNvs();
    s_params_dirty = false;
    s_last_params_save_ms = now;
  }
}
