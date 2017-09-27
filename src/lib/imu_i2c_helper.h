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

typedef struct {
  uint8_t address;
  uint8_t value;
} imu_register_value_t;

int imu_send_config(uint8_t i2c_address, const imu_register_value_t *config,
                    size_t n_register_values);
void imu_write_register(const uint8_t addr, const uint8_t reg,
                        const uint8_t *data, const uint8_t len);
int imu_read_registers(const uint8_t addr, uint8_t first_reg,
                       const uint8_t len, uint8_t * const data);

