//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick	Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 15/11/2016	Sakari Kapanen	Additions and adaptation to ESP8266: gravity, pitch and roll calculation
//
//=====================================================================================================
#pragma once

#define ACC_X_IDX  0
#define ACC_Y_IDX  1
#define ACC_Z_IDX  2
#define GYRO_X_IDX 3
#define GYRO_Y_IDX 4
#define GYRO_Z_IDX 5

#include <stdint.h>

void MadgwickAHRSupdateIMU(float beta, float gyroIntegrationFactor,
	int16_t data[]);

void gravityVector();
float pitchAngle();
float rollAngle();
float pitchAngleTaylor();
float rollAngleTaylor();

