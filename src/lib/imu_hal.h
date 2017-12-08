/*
 * Hardware abstraction layer enabling use of several different I2C IMUs in the
 * ESPway robot.
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

#include <i2c/i2c.h>
#include "espway_config.h"

void imu_i2c_configure(i2c_freq_t freq, int scl, int sca);
int imu_read_raw_data(int16_t * const data);
int imu_init(void);

#if IMU == IMU_MPU6050
#include "mpu6050.h"
#elif IMU == IMU_LSM6DS3
#include "lsm6ds3.h"
#endif

