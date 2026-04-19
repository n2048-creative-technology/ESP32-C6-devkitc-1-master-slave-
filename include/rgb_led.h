#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

struct RgbLed {
  rmt_channel_handle_t channel;
  rmt_encoder_handle_t encoder;
  bool ready;
};

bool rgbLedInit(RgbLed *led, gpio_num_t gpio);
void rgbLedSet(RgbLed *led, uint8_t r, uint8_t g, uint8_t b);
