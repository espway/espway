/*
 * Ultrasonic sensor driver for esp-open-rtos
 * Copyright (C) 2018  Sakari Kapanen
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
#include <stdbool.h>
#include <FreeRTOS.h>

#ifndef ULTRASONIC_SENSOR_MAX_COUNT
#define ULTRASONIC_SENSOR_MAX_COUNT 2
#endif

#ifndef ULTRASONIC_SENSOR_MAX_TIME_MS
#define ULTRASONIC_SENSOR_MAX_TIME_MS 20
#endif

#define CM_TO_US(cm) ((cm) * 58)

// Add an ultrasonic sensor on the given pin
void ultrasonic_sensor_init(uint8_t pins[], uint8_t n_pins);
// Read the given ultrasonic sensor. Blocks execution
int ultrasonic_sensor_read(uint8_t sensor_index);
