#pragma once

#include <stdint.h>

#define ROLE_MASTER 1
#define ROLE_SLAVE 2

#ifndef NODE_ROLE
#define NODE_ROLE ROLE_MASTER
#endif

// ESP-NOW radio configuration
static const uint8_t RADIO_CHANNEL = 6;

// Master pins (ESP32-C6 ADC1)
static const uint8_t POT_SPEED_GPIO = 0;
static const uint8_t POT_PERIOD_GPIO = 1;

// Slave pins
static const uint8_t PWM_GPIO = 5;
// Onboard addressable RGB LED (verify GPIO for your ESP32-C6-DevKitC-1).
static const uint8_t LED_GPIO = 8;

// Ranges and defaults
static const float SPEED_MIN = 0.0f;
static const float SPEED_MAX = 1.0f;

static const float PROBABILITY_MIN = 0.0f;
static const float PROBABILITY_MAX = 1.0f;

// Master behavior
static const uint32_t HEARTBEAT_INTERVAL_MS = 1000;
static const uint32_t LED_BLINK_INTERVAL_MS = 1000;
static const uint8_t LED_RED_LEVEL = 64;
static const uint8_t LED_GREEN_LEVEL = 64;
static const uint8_t LED_BLUE_LEVEL = 64;
static const uint8_t LED_WHITE_LEVEL = 255;
static const uint8_t LED_DATA_LEVEL = 10;

static const uint32_t LED_PARAM_FLASH_INTERVAL_MS = 120;
static const uint8_t LED_PARAM_FLASH_COUNT = 2;
static const uint32_t PARAMS_SAVE_DEBOUNCE_MS = 500;
static const uint32_t MASTER_LED_FLASH_MS = 80;
static const uint32_t MAC_LOG_INTERVAL_MS = 3000;
static const float SPEED_SEND_THRESHOLD = 0.01f;
static const float PROBABILITY_SEND_THRESHOLD = 0.01f;
static const uint32_t SERIAL_BAUD_RATE = 115200;
static const uint32_t SERIAL_RX_BUF = 1024;
static const uint32_t SERIAL_TX_BUF = 0;
// Master UI (OLED + encoder)
static const uint8_t I2C_SDA_GPIO = 6;
static const uint8_t I2C_SCL_GPIO = 7;
static const uint8_t OLED_I2C_ADDR = 0x3C;
static const uint8_t ENCODER_A_GPIO = 2;
static const uint8_t ENCODER_B_GPIO = 3;
static const uint8_t ENCODER_BTN_GPIO = 4;
static const uint32_t OLED_REFRESH_MS = 200;
static const uint32_t ENCODER_DEBOUNCE_MS = 30;
static const float ENCODER_SPEED_STEP = 0.01f;
static const float ENCODER_PROB_STEP = 0.01f;

// Slave behavior
static const uint32_t RX_TIMEOUT_MS = 3000;
static const uint32_t PWM_FREQ_HZ = 20000;
static const uint8_t PWM_RES_BITS = 11;
static const uint8_t PWM_CHANNEL = 0;
static const uint8_t PWM_TIMER = 0;
static const float SPEED_SLEW_PER_SEC = 1.5f;
static const uint32_t PROBABILITY_UPDATE_INTERVAL_MS = 1000;
static const uint8_t RELAY_TTL_DEFAULT = 3;
static const uint32_t HELLO_INTERVAL_MS = 1000;
static const uint32_t DISCOVER_INTERVAL_MS = 500;
static const uint32_t DISCOVER_JITTER_MS = 100;
static const uint32_t SLAVE_HELLO_INTERVAL_MS = 200;
static const uint8_t SLAVE_HELLO_BURST_COUNT = 5;
static const uint32_t SLAVE_HELLO_BURST_INTERVAL_MS = 50;
static const uint32_t SLAVE_REJOIN_BURST_INTERVAL_MS = 1000;
static const uint32_t SLAVE_HELLO_JITTER_MS = 100;
static const uint32_t SLAVE_BURST_JITTER_MS = 20;
static const float LED_PARAM_EPS = 0.0005f;
static const uint32_t PEER_STALE_MS = 15000;
static const uint32_t PEER_REMOVE_MS = 60000;
static const uint8_t PEER_CONFIRM_SAMPLES = 1;
static const uint32_t PEER_CHECK_INTERVAL_MS = 500;
