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

#include "i2c/i2c.h"
#include "mpu6050.h"

static void mpu_write_register(const uint8_t addr,
    const uint8_t reg, const uint8_t value, bool stop) {
    i2c_slave_write(MPU_ADDR, &reg, &value, 1);
}

int mpu_read_registers(const uint8_t addr,
    uint8_t first_reg, const uint8_t len, uint8_t * const data) {
    return i2c_slave_read(MPU_ADDR, &first_reg, data, len);
}

int mpu_read_int_status(const uint8_t addr) {
    uint8_t tmp = 0;
    mpu_read_registers(addr, MPU_INT_STATUS, 1, &tmp);
    return tmp;
}

int mpu_read_raw_data(const uint8_t addr,
    int16_t * const data) {
    uint8_t *my_data = (uint8_t *)data;
    uint8_t buf[14];
    int ret = mpu_read_registers(MPU_ADDR, MPU_ACCEL_XOUT_H, 14, buf);
    if (ret != 0) { return ret; }

    my_data[0] = buf[1]; my_data[1] = buf[0];      // ACC_X
    my_data[2] = buf[3]; my_data[3] = buf[2];      // ACC_Y
    my_data[4] = buf[5]; my_data[5] = buf[4];      // ACC_Z
    my_data[6] = buf[9]; my_data[7] = buf[8];      // GYRO_X
    my_data[8] = buf[11]; my_data[9] = buf[10];    // GYRO_Y
    my_data[10] = buf[13]; my_data[11] = buf[12];  // GYRO_Z

    return ret;
}

bool mpu_init(void) {
    // Wake up
    mpu_write_register(MPU_ADDR, MPU_PWR_MGMT_1,
        MPU_CLK_PLL_ZGYRO | MPU_TEMP_DIS, false);
    // 1000 Hz sampling
    mpu_write_register(MPU_ADDR, MPU_SMPRT_DIV, MPU_RATE, false);
    // 188 Hz LPF
    mpu_write_register(MPU_ADDR, MPU_CONFIG, 1, false);
    // Gyroscope min sensitivity
    mpu_write_register(MPU_ADDR, MPU_GYRO_CONFIG, 3 << 3, false);
    // Accel max sensitivity
    mpu_write_register(MPU_ADDR, MPU_ACCEL_CONFIG, 0, false);
    // Data ready interrupt on
    mpu_write_register(MPU_ADDR, MPU_INT_ENABLE, 1, false);
    // Check if the MPU still responds with its own address
    uint8_t addr = 0;
    int ret = mpu_read_registers(MPU_ADDR, MPU_WHO_AM_I, 1, &addr);
    return ret == 0 && addr == MPU_ADDR;
}

bool mpu_set_gyro_offsets(int16_t *offsets) {
    uint8_t data[] = {
        offsets[0] >> 8, offsets[0] & 0xff,
        offsets[1] >> 8, offsets[1] & 0xff,
        offsets[2] >> 8, offsets[2] & 0xff
    };
    uint8_t reg_addr = MPU_XG_OFFS_USRH;
    return i2c_slave_write(MPU_ADDR, &reg_addr, data, 6) == 0;
}

void mpu_go_to_sleep(void) {
    mpu_write_register(MPU_ADDR, MPU_PWR_MGMT_1, 1 << 6, true);
}

