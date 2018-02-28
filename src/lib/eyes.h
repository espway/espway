/*
 * Firmware for a segway-style robot using ESP8266.
 * Copyright (C) 2017  Sakari Kapanen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "ws2812_i2s/ws2812_i2s.h"

typedef ws2812_pixel_t color_t;

void set_both_eyes(const color_t color);
void eyes_init();

extern const color_t RED;
extern const color_t YELLOW;
extern const color_t GREEN;
extern const color_t BLUE;
extern const color_t LILA;
extern const color_t BLACK;

