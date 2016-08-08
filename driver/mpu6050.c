/*
A lightweight library for reading and processing motion information
from a MPU-6050 sensor.

The MIT License (MIT)
Copyright (c) 2016 Sakari Kapanen

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ets_sys.h"

#include "i2c_master.h"
#include "i2c_helper.h"
#include "mpu6050.h"

LOCAL int ICACHE_FLASH_ATTR mpuWriteRegister(const uint8_t addr,
    const uint8_t reg, const uint8_t value, const bool stop) {
    uint8_t regValue[] = { reg, value };
    i2c_master_start();
    bool ret = i2c_master_transmitTo(addr) &&
        i2c_master_writeBytes(regValue, 2);
    if (stop) i2c_master_stop();
    return ret ? 0 : -1;
}

LOCAL int ICACHE_FLASH_ATTR mpuReadRegisters(const uint8_t addr,
    const uint8_t firstReg, const uint8_t len, uint8_t * const data) {
    i2c_master_start();
    bool ret = i2c_master_transmitTo(addr) &&
        i2c_master_writeBytes((uint8_t *)&firstReg, 1);
    i2c_master_start();
    ret = ret && i2c_master_receiveFrom(addr) &&
        i2c_master_readBytes(data, len);
    i2c_master_stop();
    return ret ? 0 : -1;
}

int ICACHE_FLASH_ATTR mpuReadIntStatus(const uint8_t addr) {
    uint8_t tmp;
    mpuReadRegisters(addr, MPU_INT_STATUS, 1, &tmp);
    return tmp;
}

int ICACHE_FLASH_ATTR mpuReadRawData(const uint8_t addr, int16_t * const data) {
    uint8_t *myData = (uint8_t *)data;
    uint8_t reg[] = { MPU_ACCEL_XOUT_H };
    i2c_master_start();
    bool ret = i2c_master_transmitTo(addr) &&
        i2c_master_writeBytes(reg, 1);
    i2c_master_start();
    ret = ret && i2c_master_receiveFrom(addr);
    for (int8_t i = 0; i < 6; i++) {
        if (!ret) break;
        *(myData + 1) = i2c_master_readNextByte(true);
        *myData = i2c_master_readNextByte(true);
        myData += 2;
    }
    *(myData + 1) = i2c_master_readNextByte(false);
    *myData = i2c_master_readNextByte(false);

    i2c_master_stop();
    return ret ? 0 : -1;
}

void ICACHE_FLASH_ATTR mpuApplyOffsets(int16_t * const data,
    const int16_t * const offsets) {
    for (uint8_t i = 0; i < 7; i++) {
        data[i] += offsets[i];
    }
}

int ICACHE_FLASH_ATTR mpuSetup(const uint8_t addr,
    const mpuconfig * const config) {
    int status;
    // Wake up and disable temperature measurement if asked for
    status = mpuWriteRegister(addr, MPU_PWR_MGMT_1,
        MPU_CLK_PLL_ZGYRO | (config->disableTemp ? MPU_TEMP_DIS : 0), false);
    if (status != 0) return status;
    // Configure the low pass filter
    if (config->lowpass > 6) return -10;
    status = mpuWriteRegister(addr, MPU_CONFIG, config->lowpass, false);
    if (status != 0) return status;
    // Configure gyro and accelerometer sensitivities
    if (config->gyroRange > 3) return -11;
    status = mpuWriteRegister(addr, MPU_GYRO_CONFIG,
        config->gyroRange << 3, false);
    if (status != 0) return status;
    if (config->accelRange > 3) return -12;
    status = mpuWriteRegister(addr, MPU_ACCEL_CONFIG,
        config->accelRange << 3, false);
    if (status != 0) return status;
    // Configure the sample rate
    status = mpuWriteRegister(addr, MPU_SMPRT_DIV, config->sampleRateDivider,
        false);
    if (status != 0) return status;
    // Configure the interrupt
    status = mpuWriteRegister(addr, MPU_INT_PIN_CFG,
        (config->intOpenDrain ? MPU_INT_OPEN : 0) |
        (config->intActiveLow ? MPU_INT_LEVEL : 0) |
        MPU_INT_RD_CLEAR, true);
    if (status != 0) return status;
    status = mpuWriteRegister(addr, MPU_INT_ENABLE,
        config->enableInterrupt ? MPU_DATA_RDY_EN : 0, true);
    if (status != 0) return status;
    uint8_t id;
    status = mpuReadRegisters(addr, MPU_WHO_AM_I, 1, &id);
    if (status != 0) return status;
    return id == addr ? 0 : -1;
}

// Pitch calculation by means of the complementary filter

LOCAL const int16_t MPU_GYRO_RANGE[] = { 250, 500, 1000, 2000 };
LOCAL const int8_t MPU_ACCEL_RANGE[] = { 2, 4, 8, 16 };
LOCAL const int16_t ANGLE_SCALE_FACTOR = 256;
void ICACHE_FLASH_ATTR mpuSetupFilter(const mpuconfig * const config,
    mpufilter * const filter, const int16_t alpha) {
    filter->alpha = alpha;
    filter->alphaComplement = 512 - filter->alpha;

    int16_t g = (INT16_MAX / MPU_ACCEL_RANGE[config->accelRange]) >> 8;
    filter->g2 = g*g;
    filter->gThresh = g*g * 3 / 2;

    int32_t c = (int32_t)(1 + config->sampleRateDivider) * g * 314 *
                MPU_GYRO_RANGE[config->gyroRange] / INT16_MAX,
            d = 18000L * 1000 / ANGLE_SCALE_FACTOR;
    filter->gyroDivider = d / c;
}

void ICACHE_FLASH_ATTR mpuUpdatePitch(mpufilter * const filter,
    int16_t * const data, int16_t * const pitch) {
    // Use only the 8 most significant bits of the accelerometer readings to
    // take advantage from single-instruction multiplication (muls).
    // The readings won't have more than 8 bits of real information anyway.
    int8_t *data8 = (int8_t*)data;
    int8_t ax = data8[(MPU_ACC_X << 1) + 1],
           ay = data8[(MPU_ACC_Y << 1) + 1],
           az = data8[(MPU_ACC_Z << 1) + 1];
    int16_t gy = data[MPU_GYRO_Y];

    int16_t ayz2 = ay*ay + az*az,
            ax2 = ax*ax, a2 = ayz2 + ax2;

    int16_t gyroTerm = *pitch + gy / filter->gyroDivider;
    int16_t accTerm = 0;
    if (a2 < filter->gThresh) {
        accTerm = ANGLE_SCALE_FACTOR * ax;
#ifndef MPU6050_FIRST_ORDER
        accTerm = (int32_t)accTerm * (filter->g2 + ax2/6) / filter->g2;
#endif
        *pitch = ((int32_t)filter->alphaComplement * gyroTerm -
            (int32_t)filter->alpha * accTerm) / 512;
    } else {
        *pitch = gyroTerm;
    }
}

