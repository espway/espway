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

#include <ets_sys.h>
#include <esp8266.h>
#include <osapi.h>

#include "i2c.h"
#include "mpu6050.h"

int ICACHE_FLASH_ATTR mpu_write_register(const uint8_t addr,
    const uint8_t reg, const uint8_t value, const bool stop) {
    uint8_t reg_value[] = { reg, value };
    i2c_start();
    bool ret = i2c_transmit_to(addr);
    ret &= i2c_write_bytes(reg_value, 2);
    if (stop) i2c_stop();
    return ret ? 0 : -1;
}

int ICACHE_FLASH_ATTR mpu_read_registers(const uint8_t addr,
    const uint8_t first_reg, const uint8_t len, uint8_t * const data) {
    i2c_start();
    bool ret = i2c_transmit_to(addr);
    i2c_write_byte(first_reg);
    ret &= i2c_check_ack();
    i2c_start();
    ret &= i2c_receive_from(addr);
    ret &= i2c_read_bytes(data, len);
    i2c_stop();
    return ret ? 0 : -1;
}

int ICACHE_FLASH_ATTR mpu_read_int_status(const uint8_t addr) {
    uint8_t tmp = 0;
    mpu_read_registers(addr, MPU_INT_STATUS, 1, &tmp);
    return tmp;
}

int ICACHE_FLASH_ATTR mpu_read_raw_data(const uint8_t addr,
    int16_t * const data) {
    uint8_t *my_data = (uint8_t *)data;
    uint8_t reg = MPU_ACCEL_XOUT_H;
    i2c_start();
    bool ret = i2c_transmit_to(addr) &&
        i2c_write_bytes(&reg, 1);
    i2c_start();
    ret = ret && i2c_receive_from(addr);

    if (ret) {
        // Read accelerometer data
        for (int8_t i = 0; i < 3; i++) {
            *(my_data + 1) = i2c_read_byte();
            i2c_send_ack(true);
            *my_data = i2c_read_byte();
            i2c_send_ack(true);
            my_data += 2;
        }
        // Read and discard temperature data
        i2c_read_byte();
        i2c_send_ack(true);
        i2c_read_byte();
        i2c_send_ack(true);
        // Read gyroscope data
        for (int8_t i = 0; i < 2; i++) {
            *(my_data + 1) = i2c_read_byte();
            i2c_send_ack(true);
            *my_data = i2c_read_byte();
            i2c_send_ack(true);
            my_data += 2;
        }
        *(my_data + 1) = i2c_read_byte();
        i2c_send_ack(true);
        *my_data = i2c_read_byte();
        i2c_send_ack(false);
    }

    i2c_stop();
    return ret ? 0 : -1;
}

bool ICACHE_FLASH_ATTR mpu_init(void) {
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
    mpu_read_registers(MPU_ADDR, MPU_WHO_AM_I, 1, &addr);
    return addr == MPU_ADDR;
}

bool ICACHE_FLASH_ATTR mpu_set_gyro_offsets(int16_t *offsets) {
    bool ret = i2c_transmit_to(MPU_ADDR);
    if (!ret) return ret;
    uint8_t data[] = {
        MPU_XG_OFFS_USRH,
        offsets[0] >> 8, offsets[0] & 0xff,
        offsets[1] >> 8, offsets[1] & 0xff,
        offsets[2] >> 8, offsets[2] & 0xff
    };
    return i2c_write_bytes(data, 7);
}

void ICACHE_FLASH_ATTR mpu_go_to_sleep(void) {
    mpu_write_register(MPU_ADDR, MPU_PWR_MGMT_1, 1 << 6, true);
}

