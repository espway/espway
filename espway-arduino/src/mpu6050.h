/*
    A lightweight library for reading and processing motion information
    from a MPU-6050 sensor.
    Copyright (C) 2016  Sakari Kapanen

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

#pragma once

#include <stdint.h>

#define MPU_ADDR 0x68
#define MPU_BITRATE 400
#define MPU_RATE 0

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

#define MPU_XG_OFFS_USRH 0x13
#define MPU_XG_OFFS_USRL 0x14
#define MPU_YG_OFFS_USRH 0x15
#define MPU_YG_OFFS_USRL 0x16
#define MPU_ZG_OFFS_USRH 0x17
#define MPU_ZG_OFFS_USRL 0x18

int mpu_read_registers(const uint8_t addr,
    uint8_t firstReg, const uint8_t len, uint8_t * const data);
int mpu_read_int_status(const uint8_t addr);
int mpu_read_raw_data(const uint8_t addr, int16_t * const data);

bool mpu_init(int *ret);
void mpu_go_to_sleep(void);
bool mpu_set_gyro_offsets(int16_t * offsets);

