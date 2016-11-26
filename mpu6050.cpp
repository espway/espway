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

#include <Wire.h>

#include "mpu6050.h"

static int mpuWriteRegister(const uint8_t addr,
    const uint8_t reg, const uint8_t value, const bool stop) {
    Wire.beginTransmission(addr);
    if (Wire.write(reg) != 1) return -1;
    if (Wire.write(value) != 1) return -2;
    return Wire.endTransmission(stop);
}

static int mpuReadRegisters(const uint8_t addr,
    const uint8_t firstReg, const uint8_t len, uint8_t * const data) {
    int status;
    Wire.beginTransmission(addr);
    if (Wire.write(firstReg) != 1) return -1;
    status = Wire.endTransmission(false);
    if (status != 0) return status;
    if (Wire.requestFrom(addr, len, (uint8_t)true) != len) return -2;
    for (uint8_t i = 0; i < len; ++i) data[i] = Wire.read();
    return 0;
}

int mpuReadIntStatus(const uint8_t addr) {
    uint8_t tmp;
    mpuReadRegisters(addr, MPU_INT_STATUS, 1, &tmp);
    return tmp;
}

int mpuReadRawData(const uint8_t addr, int16_t * const data) {
    uint8_t *myData = (uint8_t *)data;
    int status;
    Wire.beginTransmission(addr);
    if (Wire.write(MPU_ACCEL_XOUT_H) != 1) return -1;
    status = Wire.endTransmission(false);
    if (status != 0) return status;
    if (Wire.requestFrom(addr, (uint8_t)14, (uint8_t)true) != 14) return -2;
    for (int8_t i = 0; i < 7; ++i) {
        *(myData + 1) = Wire.read();
        *myData = Wire.read();
        myData += 2;
    }
    return 0;
}

void mpuApplyOffsets(int16_t * const data,
    const int16_t * const offsets) {
    for (uint8_t i = 0; i < 7; ++i) {
        data[i] += offsets[i];
    }
}

static const int16_t MPU_GYRO_RANGE[] = { 250, 500, 1000, 2000 };
static const int8_t MPU_ACCEL_RANGE[] = { 2, 4, 8, 16 };

int mpuSetup(const uint8_t addr,
    mpuconfig * const config) {
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

    double sampleTime = (1 + config->sampleRateDivider) / 1000.0f;
    config->gyroScale = M_PI * MPU_GYRO_RANGE[config->gyroRange] /
        (180.0f * INT16_MAX);
    config->gyroIntegrationFactor = 0.5f * config->gyroScale * sampleTime;
    config->correctedBeta = config->beta / (0.5f * config->gyroScale);

    return id == addr ? 0 : -1;
}

void mpuUpdateQuaternion(const mpuconfig * const config,
    int16_t * const data) {
    MadgwickAHRSupdateIMU(config->correctedBeta, config->gyroIntegrationFactor,
        data[MPU_GYRO_X], data[MPU_GYRO_Y], data[MPU_GYRO_Z],
        data[MPU_ACC_X], data[MPU_ACC_Y], data[MPU_ACC_Z]);
}

