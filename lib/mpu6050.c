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

#include "ets_sys.h"

#include "i2c_master.h"
#include "mpu6050.h"

static int mpuWriteRegister(const uint8_t addr,
    const uint8_t reg, const uint8_t value, const bool stop) {
    uint8_t regValue[] = { reg, value };
    i2c_master_start();
    bool ret = i2c_master_transmitTo(addr) &&
        i2c_master_writeBytes(regValue, 2);
    if (stop) i2c_master_stop();
    return ret ? 0 : -1;
}

static int mpuReadRegisters(const uint8_t addr,
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

int mpuReadIntStatus(const uint8_t addr) {
    uint8_t tmp;
    mpuReadRegisters(addr, MPU_INT_STATUS, 1, &tmp);
    return tmp;
}

int mpuReadRawData(const uint8_t addr, int16_t * const data) {
    uint8_t *myData = (uint8_t *)data;
    uint8_t reg = MPU_ACCEL_XOUT_H;
    i2c_master_start();
    bool ret = i2c_master_transmitTo(addr) &&
        i2c_master_writeBytes(&reg, 1);
    i2c_master_start();
    ret = ret && i2c_master_receiveFrom(addr);

    if (ret) {
        for (int8_t i = 0; i < 3; i++) {
            *(myData + 1) = i2c_master_readNextByte(true);
            *myData = i2c_master_readNextByte(true);
            myData += 2;
        }
        i2c_master_readNextByte(true);
        i2c_master_readNextByte(true);
        for (int8_t i = 0; i < 2; i++) {
            *(myData + 1) = i2c_master_readNextByte(true);
            *myData = i2c_master_readNextByte(true);
            myData += 2;
        }
        *(myData + 1) = i2c_master_readNextByte(true);
        *myData = i2c_master_readNextByte(false);
    }

    i2c_master_stop();
    return ret ? 0 : -1;
}

void ICACHE_FLASH_ATTR mpuApplyOffsets(int16_t * const data,
    const int16_t * const offsets) {
    for (uint8_t i = 0; i < 6; ++i) {
        data[i] += offsets[i];
    }
}

static const int16_t MPU_GYRO_RANGE[] = { 250, 500, 1000, 2000 };
static const int8_t MPU_ACCEL_RANGE[] = { 2, 4, 8, 16 };

int ICACHE_FLASH_ATTR mpuSetup(const uint8_t addr,
    mpuconfig * const config) {
    int status;
    // Wake up and disable temperature measurement if asked for
    status = mpuWriteRegister(addr, MPU_PWR_MGMT_1,
        MPU_CLK_PLL_ZGYRO | MPU_TEMP_DIS, false);
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

    float sampleTime = (1 + config->sampleRateDivider) / 1000.0f;
    config->gyroScale = M_PI * MPU_GYRO_RANGE[config->gyroRange] /
        (180.0f * INT16_MAX);
    config->gyroIntegrationFactor = 0.5f * config->gyroScale * sampleTime;
    config->correctedBeta = config->beta / (0.5f * config->gyroScale);

    float gyroScale = 2.0f * M_PI / 180.0f * MPU_GYRO_RANGE[config->gyroRange];
    config->gyroIntegrationFactor_fix = float_to_q16(0.5f * gyroScale * sampleTime);
    config->correctedBeta_fix = float_to_q16(config->beta / (0.5f * gyroScale));

    return id == addr ? 0 : -1;
}

void ICACHE_FLASH_ATTR mpuUpdateQuaternion(const mpuconfig * const config,
    int16_t * const data, quaternion * const q) {
    MadgwickAHRSupdateIMU(config->correctedBeta, config->gyroIntegrationFactor,
        data, q);
}

void ICACHE_FLASH_ATTR mpuUpdateQuaternion_fix(const mpuconfig * const config,
    int16_t * const data, quaternion_fix * const q) {
    MadgwickAHRSupdateIMU_fix(
        config->correctedBeta_fix,
        config->gyroIntegrationFactor_fix,
        data, q);
}

