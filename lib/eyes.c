/*
    Firmware for a segway-style robot using ESP8266.
    Copyright (C) 2017  Sakari Kapanen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "eyes.h"

#include "ws2812_i2s.h"

void ICACHE_FLASH_ATTR set_both_eyes(const color_t color) {
    uint8_t buf[] = {
        color.g, color.r, color.b,
        color.g, color.r, color.b
    };
    ws2812_push(buf, 6);
}

void ICACHE_FLASH_ATTR eyes_init(void) {
    ws2812_init();
}

