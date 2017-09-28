/*
 * A lightweight library for reading and processing motion information
 * from a MPU-6050 sensor.
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

#if IMU == IMU_MPU6050

// Register addresses and bits as per the MPU-6050 datasheet
// http://43zrtwysvxb2gf29r5o0athu.wpengine.netdna-cdn.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

#define MPU_PWR_MGMT_1 0x6B
#define MPU_TEMP_DIS (1 << 3)
#define MPU_CLK_PLL_ZGYRO 3

#define MPU_CONFIG 0x1A
#define MPU_SMPRT_DIV 0x19

#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C

#define MPU_INT_PIN_CFG 0x37
#define MPU_INT_LEVEL (1 << 7)
#define MPU_INT_OPEN (1 << 6)
#define MPU_INT_RD_CLEAR (1 << 4)

#define MPU_INT_ENABLE 0x38
#define MPU_DATA_RDY_EN 1
#define MPU_MOT_EN (1 << 6)

#define MPU_INT_STATUS 0x3A
#define MPU_DATA_RDY_INT 1

#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_ACCEL_XOUT_L 0x3C
#define MPU_ACCEL_YOUT_H 0x3D
#define MPU_ACCEL_YOUT_L 0x3E
#define MPU_ACCEL_ZOUT_H 0x3F
#define MPU_ACCEL_ZOUT_L 0x40

#define MPU_TEMP_OUT_H 0x41
#define MPU_TEMP_OUT_L 0x42

#define MPU_GYRO_XOUT_H 0x43
#define MPU_GYRO_XOUT_L 0x44
#define MPU_GYRO_YOUT_H 0x45
#define MPU_GYRO_YOUT_L 0x46
#define MPU_GYRO_ZOUT_H 0x47
#define MPU_GYRO_ZOUT_L 0x48

#define MPU_WHO_AM_I 0x75

static const imu_register_value_t MPU_CONFIG_VALUES[] = {
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
};

int imu_read_raw_data(int16_t * const data)
{
  uint8_t *my_data = (uint8_t *)data;
  uint8_t buf[14];
  int ret = imu_read_registers(MPU_ADDR, MPU_ACCEL_XOUT_H, buf, 14);
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
}

int imu_init(void)
{
  imu_i2c_init(I2C_FREQ_400K);

  int ret = imu_send_config(MPU_ADDR, MPU_CONFIG_VALUES,
                            sizeof(MPU_CONFIG_VALUES) / sizeof(MPU_CONFIG_VALUES[0]));
  if (ret != 0)
  {
    return ret;
  }

  uint8_t addr = 0;
  ret = imu_read_register(MPU_ADDR, MPU_WHO_AM_I, &addr);
  if (ret != 0)
  {
    return ret;
  }
  if (addr != MPU_ADDR)
  {
    return -1;
  }

  return 0;
}

#endif

