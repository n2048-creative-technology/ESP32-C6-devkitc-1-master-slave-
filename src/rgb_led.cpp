#include "../include/rgb_led.h"

#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "rgb_led";

static rmt_symbol_word_t makeSymbol(uint32_t high_ticks, uint32_t low_ticks) {
  rmt_symbol_word_t symbol = {};
  symbol.level0 = 1;
  symbol.duration0 = high_ticks;
  symbol.level1 = 0;
  symbol.duration1 = low_ticks;
  return symbol;
}

bool rgbLedInit(RgbLed *led, gpio_num_t gpio) {
  if (led == nullptr) {
    return false;
  }
  led->ready = false;
  led->channel = nullptr;
  led->encoder = nullptr;

  rmt_tx_channel_config_t tx_config = {};
  tx_config.gpio_num = gpio;
  tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_config.resolution_hz = 10 * 1000 * 1000;
  tx_config.mem_block_symbols = 64;
  tx_config.trans_queue_depth = 1;
  tx_config.intr_priority = 0;
  tx_config.flags.invert_out = 0;
  tx_config.flags.with_dma = 0;
  tx_config.flags.io_loop_back = 0;
  tx_config.flags.io_od_mode = 0;
  tx_config.flags.allow_pd = 0;

  esp_err_t ret = rmt_new_tx_channel(&tx_config, &led->channel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "rmt_new_tx_channel failed err=%d", (int)ret);
    return false;
  }

  rmt_bytes_encoder_config_t enc_cfg = {};
  enc_cfg.bit0 = makeSymbol(4, 9);
  enc_cfg.bit1 = makeSymbol(8, 5);
  enc_cfg.flags.msb_first = 1;
  ret = rmt_new_bytes_encoder(&enc_cfg, &led->encoder);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "rmt_new_bytes_encoder failed err=%d", (int)ret);
    return false;
  }

  ret = rmt_enable(led->channel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "rmt_enable failed err=%d", (int)ret);
    return false;
  }

  led->ready = true;
  return true;
}

void rgbLedSet(RgbLed *led, uint8_t r, uint8_t g, uint8_t b) {
  if (led == nullptr || !led->ready) {
    return;
  }
  uint8_t data[3] = {g, r, b};
  rmt_transmit_config_t tx_cfg = {};
  tx_cfg.loop_count = 0;
  tx_cfg.flags.eot_level = 0;
  tx_cfg.flags.queue_nonblocking = 1;
  esp_err_t ret = rmt_transmit(led->channel, led->encoder, data, sizeof(data),
                               &tx_cfg);
  if (ret == ESP_OK) {
    rmt_tx_wait_all_done(led->channel, 10);
  } else if (ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "rmt_transmit failed err=%d", (int)ret);
  }
}
