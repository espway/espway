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

int ICACHE_FLASH_ATTR mpuWriteRegister(const uint8_t addr,
    const uint8_t reg, const uint8_t value, const bool stop) {
    uint8_t regValue[] = { reg, value };
    i2c_start();
    bool ret = i2c_transmitTo(addr);
    ret &= i2c_writeBytes(regValue, 2);
    if (stop) i2c_stop();
    return ret ? 0 : -1;
}

int ICACHE_FLASH_ATTR mpuReadRegisters(const uint8_t addr,
    const uint8_t firstReg, const uint8_t len, uint8_t * const data) {
    i2c_start();
    bool ret = i2c_transmitTo(addr);
    os_printf("\n0: %u\n", ret);
    //ret &= i2c_writeByte(firstReg);
    i2c_writeByte(firstReg);
    i2c_checkAck();
    os_printf("1: %u\n", ret);
    i2c_start();
    ret &= i2c_receiveFrom(addr);
    os_printf("2: %u\n", ret);
    ret &= i2c_readBytes(data, len);
    os_printf("3: %u\n", ret);
    i2c_stop();
    return ret ? 0 : -1;
}

int ICACHE_FLASH_ATTR mpuReadIntStatus(const uint8_t addr) {
    uint8_t tmp;
    mpuReadRegisters(addr, MPU_INT_STATUS, 1, &tmp);
    return tmp;
}

int ICACHE_FLASH_ATTR mpuReadRawData(const uint8_t addr, int16_t * const data) {
    uint8_t *myData = (uint8_t *)data;
    uint8_t reg = MPU_ACCEL_XOUT_H;
    i2c_start();
    bool ret = i2c_transmitTo(addr) &&
        i2c_writeBytes(&reg, 1);
    i2c_start();
    ret = ret && i2c_receiveFrom(addr);

    if (ret) {
        for (int8_t i = 0; i < 3; i++) {
            *(myData + 1) = i2c_readByte();
            i2c_send_ack(true);
            *myData = i2c_readByte();
            i2c_send_ack(true);
            myData += 2;
        }
        i2c_readByte();
        i2c_send_ack(true);
        i2c_readByte();
        i2c_send_ack(true);
        for (int8_t i = 0; i < 2; i++) {
            *(myData + 1) = i2c_readByte();
            i2c_send_ack(true);
            *myData = i2c_readByte();
            i2c_send_ack(true);
            myData += 2;
        }
        *(myData + 1) = i2c_readByte();
        i2c_send_ack(true);
        *myData = i2c_readByte();
        i2c_send_ack(false);
    }

    i2c_stop();
    return ret ? 0 : -1;
}

