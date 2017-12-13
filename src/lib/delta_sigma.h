/*
 * Delta sigma modulator for ESP8266
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

#ifndef DELTA_SIGMA_MAX_PINS
#define DELTA_SIGMA_MAX_PINS 2
#endif

void delta_sigma_set_duty(uint8_t channel, uint32_t duty);
void delta_sigma_start(uint32_t period, uint32_t range, uint8_t pins[],
                       uint8_t n_pins);
