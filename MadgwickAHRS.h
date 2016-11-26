//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 15/11/2016   Sakari Kapanen  Additions and adaptation to ESP8266: gravity, pitch and roll calculation
//
//=====================================================================================================
#pragma once

#include <stdint.h>

void MadgwickAHRSupdateIMU(float beta, float gyroIntegrationFactor, int16_t gx, int16_t gy, int16_t gz, int16_t ax, int16_t ay, int16_t az);

void gravityVector();
double pitchAngle();
double rollAngle();

