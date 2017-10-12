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

#include "i2c/i2c.h"
#include "imu_hal.h"
#include "imu_i2c_helper.h"

#if IMU == IMU_LSM6DS3

#define LSM6DS3_WHO_AM_I 0x0f
#define LSM6DS3_WHO_AM_I_VALUE 0x69

static const imu_register_value_t LSM6DS3_CONFIG_VALUES[] = {
  /*
  // Wake up
  {MPU_PWR_MGMT_1, MPU_CLK_PLL_ZGYRO | MPU_TEMP_DIS},
  // 1000 Hz sample rate
  {MPU_SMPRT_DIV, 0},
  // 188 Hz LPF
  {MPU_CONFIG, 1},
  // Gyroscope min sensitivity
  {MPU_GYRO_CONFIG, 3 << 3},
  // Accel max sensitivity
  {MPU_ACCEL_CONFIG, 0},
  // Data ready interrupt on
  {MPU_INT_PIN_CFG, 0x10},
  // Enable interrupts
  {MPU_INT_ENABLE, 1}
  */
};

int imu_read_raw_data(int16_t * const data)
{
  return -1;
  /*
  uint8_t *my_data = (uint8_t *)data;
  uint8_t buf[14];
  int ret = imu_read_registers(IMU_ADDR, MPU_ACCEL_XOUT_H, buf, 14);
  if (ret != 0)
  {
    return ret;
  }

  my_data[0] = buf[1]; my_data[1] = buf[0];      // ACC_X
  my_data[2] = buf[3]; my_data[3] = buf[2];      // ACC_Y
  my_data[4] = buf[5]; my_data[5] = buf[4];      // ACC_Z
  my_data[6] = buf[9]; my_data[7] = buf[8];      // GYRO_X
  my_data[8] = buf[11]; my_data[9] = buf[10];    // GYRO_Y
  my_data[10] = buf[13]; my_data[11] = buf[12];  // GYRO_Z

  return ret;
  */
}

int imu_init(void)
{
  int ret;

  imu_i2c_init(I2C_FREQ_400K);

  /*
  ret = imu_send_config(IMU_ADDR, MPU_CONFIG_VALUES,
                            sizeof(MPU_CONFIG_VALUES) / sizeof(MPU_CONFIG_VALUES[0]));
  if (ret != 0)
  {
    return ret;
  }
  */

  printf("IMU address: %x\n", IMU_ADDR);

  uint8_t whoami = 0;
  ret = imu_read_register(IMU_ADDR, LSM6DS3_WHO_AM_I, &whoami);
  if (ret != 0)
  {
    printf("Error reading IMU register\n");
    return ret;
  }
  if (whoami != LSM6DS3_WHO_AM_I_VALUE)
  {
    return -1;
  }

  return 0;
}

#endif

