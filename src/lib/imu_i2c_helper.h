/*
 * Helper for doing common tasks with I2C IMUs (and maybe other devices).
 * Copyright (C) 2017  Sakari Kapanen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include "i2c/i2c.h"

typedef struct {
  uint8_t address;
  uint8_t value;
} imu_register_value_t;

void imu_i2c_init(i2c_freq_t freq);
int imu_send_config(uint8_t i2c_address, const imu_register_value_t *config,
                    size_t n_register_values);
int imu_write_registers(uint8_t addr, uint8_t reg, const uint8_t *data,
                        uint8_t len);
int imu_read_registers(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
int imu_write_register(uint8_t addr, uint8_t reg, uint8_t value);
int imu_read_register(uint8_t addr, uint8_t reg, uint8_t *value);

