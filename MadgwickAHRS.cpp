//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
// 15/11/2016	Sakari Kapanen	Additions and adaptation to ESP8266: gravity, pitch and roll calculation
//
//=====================================================================================================

#include <math.h>

#include "MadgwickAHRS.h"

static float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

static inline float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float beta, float gyroIntegrationFactor,
	int16_t data[]) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot0, qDot1, qDot2, qDot3;
	float _2q1q1q2q2, tmpterm;

	float gx = data[GYRO_X_IDX],
		gy = data[GYRO_Y_IDX],
		gz = data[GYRO_Z_IDX];

	// Rate of change of quaternion from gyroscope
	// NOTE these values are actually double the actual values but it
	// does not matter
	qDot0 = -q1 * gx - q2 * gy - q3 * gz;
	qDot1 = q0 * gx + q2 * gz - q3 * gy;
	qDot2 = q0 * gy - q1 * gz + q3 * gx;
	qDot3 = q0 * gz + q1 * gy - q2 * gx;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (data[ACC_X_IDX] != 0 || data[ACC_Y_IDX] != 0 ||
		data[ACC_Z_IDX] != 0) {
		float ax = data[ACC_X_IDX],
			ay = data[ACC_Y_IDX],
			az = data[ACC_Z_IDX];
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax*ax + ay*ay + az*az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q1q1q2q2 = q1*q1 + q2*q2;
		tmpterm = az + _2q1q1q2q2;
		tmpterm += tmpterm;
		_2q1q1q2q2 += _2q1q1q2q2;

		// Gradient descent algorithm corrective step
		s0 = q0 * _2q1q1q2q2 + q2 * ax - q1 * ay;
		s1 = q1 * tmpterm - q3 * ax - q0 * ay;
		s2 = q2 * tmpterm + q0 * ax - q3 * ay;
		s3 = q3 * _2q1q1q2q2 - q1 * ax - q2 * ay;

		// Apply feedback step
		recipNorm = beta * invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
		qDot0 -= recipNorm * s0;
		qDot1 -= recipNorm * s1;
		qDot2 -= recipNorm * s2;
		qDot3 -= recipNorm * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot0 * gyroIntegrationFactor;
	q1 += qDot1 * gyroIntegrationFactor;
	q2 += qDot2 * gyroIntegrationFactor;
	q3 += qDot3 * gyroIntegrationFactor;

	// Normalise quaternion
	recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

float pitchAngle() {
	float half_gravx = q1*q3 - q0*q2;
	return atan(half_gravx * invSqrt(0.25f - half_gravx*half_gravx));
}

float rollAngle() {
	float half_gravy = q0*q1 + q2*q3;
	return atan(half_gravy * invSqrt(0.25f - half_gravy*half_gravy));
}

float pitchAngleTaylor() {
	// x is half of the unit gravitation vector x component gx/2
	// (to save one multiplication).
	// For the unit vector g,
	// pitch = atan(gx / sqrt(gy^2 + gz^2)) = atan(gx / sqrt(1 - gx^2)).
	// Therefore for g/2,
	// pitch = atan(gx/2 / sqrt((1/2)^2 - (gx/2)^2)
	//		 = atan(gx/2 / sqrt(1/4 - (gx/2)^2))
	// The below fuction is a Taylor expansion of the latter form.
	float x = q1*q3 - q0*q2;
	float x2 = x*x;
	return x * (2.0f + x2 * (1.33333f + x2 * 2.4f));
}

float rollAngleTaylor() {
	// See pitchAngleTaylor for explanation of the expansion
	float y = q0*q1 + q2*q3;
	float y2 = y*y;
	return y * (2.0f + y2 * (1.33333f + y2 * 2.4f));
}

