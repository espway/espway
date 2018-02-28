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

#include "eyes.h"

const color_t RED    = { .color = 0xb40000 };
const color_t YELLOW = { .color = 0xb4b400 };
const color_t GREEN  = { .color = 0x00b400 };
const color_t BLUE   = { .color = 0x0000b4 };
const color_t LILA   = { .color = 0xb400b4 };
const color_t BLACK  = { .color = 0x000000 };

void set_both_eyes(const color_t color)
{
  color_t colors[2] = { color, color };
  ws2812_i2s_update(colors, PIXEL_RGB);
}

void eyes_init()
{
  ws2812_i2s_init(2, PIXEL_RGB);
}

