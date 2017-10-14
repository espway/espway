/*
 * A lightweight library for reading and processing motion information
 * from LSM6DS3 sensor.
 * Copyright (C) 2017  Sakari Kapanen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. * See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. * If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include "i2c/i2c.h"
#include "imu_hal.h"
#include "imu_i2c_helper.h"

#if IMU == IMU_LSM6DS3

#define LSM6DS3_WHO_AM_I 0x0f
#define LSM6DS3_WHO_AM_I_VALUE 0x69

#define INT1_CTRL 0x0d
#define INT1_DRDY_G (0x01 << 1)

#define CTRL1_XL 0x10
#define BW_XL_200Hz 0x01
#define ODR_XL_1K66 (0x08 << 4)

#define CTRL2_G 0x11
#define FS_G_2000_DPS (0x11 << 2)
#define ODR_G_1K66 (0x08 << 4)

#define OUTX_L_G 0x22
#define OUTX_L_XL 0x28


static const imu_register_value_t LSM6DS3_CONFIG_VALUES[] = {
  {INT1_CTRL, INT1_DRDY_G},
  {CTRL1_XL, ODR_XL_1K66},
  {CTRL2_G, FS_G_2000_DPS | ODR_G_1K66}
};

int imu_read_raw_data(int16_t * const data)
{
  uint8_t *my_data = (uint8_t *)data;
  uint8_t tmp[12];
  int ret = imu_read_registers(IMU_ADDR, OUTX_L_G, tmp, 12);
  if (ret != 0)
  {
    return ret;
  }

  memcpy(&my_data[6], &tmp[0], 6);
  memcpy(&my_data[0], &tmp[6], 6);

  return ret;
}

int imu_init(void)
{
  int ret;

  imu_i2c_init(I2C_FREQ_500K);

  ret = imu_send_config(IMU_ADDR, LSM6DS3_CONFIG_VALUES,
                        sizeof(LSM6DS3_CONFIG_VALUES) / sizeof(LSM6DS3_CONFIG_VALUES[0]));
  if (ret != 0)
  {
    return ret;
  }

  uint8_t whoami = 0;
  ret = imu_read_register(IMU_ADDR, LSM6DS3_WHO_AM_I, &whoami);
  if (ret != 0)
  {
    return ret;
  }
  if (whoami != LSM6DS3_WHO_AM_I_VALUE)
  {
    return -1;
  }

  return 0;
}

#endif

