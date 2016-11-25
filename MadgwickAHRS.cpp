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
// 15/11/2016   Sakari Kapanen  Additions and adaptation to ESP8266: gravity, pitch and roll calculation
//
//=====================================================================================================

#include <math.h>

#include "MadgwickAHRS.h"

static float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
static float gravx = 0.0, gravy = 0.0, gravz = 0.0;

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
	int16_t gx, int16_t gy, int16_t gz,
	int16_t ax, int16_t ay, int16_t az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _q3q3az1, _2q1q1q2q2, tmpterm;

	// Rate of change of quaternion from gyroscope
	// NOTE these values are actually double the actual values but will be
	// divided by two later
	qDot1 = -q1 * gx - q2 * gy - q3 * gz;
	qDot2 = q0 * gx + q2 * gz - q3 * gy;
	qDot3 = q0 * gy - q1 * gz + q3 * gx;
	qDot4 = q0 * gz + q1 * gy - q2 * gx;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax*ax + ay*ay + az*az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q1q1q2q2 = q1*q1 + q2*q2;
		_2q1q1q2q2 += _2q1q1q2q2;
		_q3q3az1 = q3*q3 + az - 1.0f;
		tmpterm = _q3q3az1 + _2q1q1q2q2 + q0*q0;
		tmpterm += tmpterm;

		// Gradient descent algorithm corrective step
		s0 = q0 * _2q1q1q2q2 + q2 * ax - q1 * ay;
		s1 = q1 * tmpterm - q3 * ax - q0 * ay;
		s2 = q2 * tmpterm + q0 * ax - q3 * ay;
		s3 = q3 * _2q1q1q2q2 - q1 * ax - q2 * ay;

		// Apply feedback step
                double betaRecipNorm =
                    beta * invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
		qDot1 -= betaRecipNorm * s0;
		qDot2 -= betaRecipNorm * s1;
		qDot3 -= betaRecipNorm * s2;
		qDot4 -= betaRecipNorm * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * gyroIntegrationFactor;
	q1 += qDot2 * gyroIntegrationFactor;
	q2 += qDot3 * gyroIntegrationFactor;
	q3 += qDot4 * gyroIntegrationFactor;

	// Normalise quaternion
	recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void gravityVector() {
    gravx = q1*q3 - q0*q2;
    gravx += gravx;
    gravy = q0*q1 + q2*q3;
    gravy += gravy;
    gravz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}

double pitchAngle() {
    return atan(gravx * invSqrt(gravy*gravy + gravz*gravz));
}

double rollAngle() {
    return atan(gravy * invSqrt(gravx*gravx + gravz*gravz));
}

