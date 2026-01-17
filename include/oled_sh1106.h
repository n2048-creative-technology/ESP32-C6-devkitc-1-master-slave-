#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c.h"

bool oledInit(i2c_port_t port, int sda_gpio, int scl_gpio, uint8_t address);
void oledClear();
void oledDrawString(int x, int y, const char *text);
void oledUpdate();
