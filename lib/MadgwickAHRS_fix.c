//=============================================================================
// MadgwickAHRS.c
//=============================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date	        Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
// 19/02/2012   SOH Madgwick    Magnetometer measurement is normalised
// 14/12/2016   Sakari Kapanen  Fixed point version of the algorithm
//
//=============================================================================

#include "MadgwickAHRS_fix.h"

//-----------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU_fix(q16 beta, q16 gyroIntegrationFactor,
    int16_t data[], quaternion_fix * const q) {
    q16 recipNorm;
    q16 q0, q1, q2, q3;
    q16 s0, s1, s2, s3;
    q16 qDot0, qDot1, qDot2, qDot3;
    q16 _2q1q1q2q2, tmpterm;

    q0 = q->q0;
    q1 = q->q1;
    q2 = q->q2;
    q3 = q->q3;

    q16 gx = data[GYRO_X_IDX],
        gy = data[GYRO_Y_IDX],
        gz = data[GYRO_Z_IDX];

    // Rate of change of quaternion from gyroscope
    // NOTE these values are actually double the actual values but it
    // does not matter
    qDot0 = -q16_mul(q1, gx) - q16_mul(q2, gy) - q16_mul(q3, gz);
    qDot1 = q16_mul(q0, gx) + q16_mul(q2, gz) - q16_mul(q3, gy);
    qDot2 = q16_mul(q0, gy) - q16_mul(q1, gz) + q16_mul(q3, gx);
    qDot3 = q16_mul(q0, gz) + q16_mul(q1, gy) - q16_mul(q2, gx);

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (data[ACC_X_IDX] != 0 || data[ACC_Y_IDX] != 0 || data[ACC_Z_IDX] != 0) {
        q16 ax = data[ACC_X_IDX],
            ay = data[ACC_Y_IDX],
            az = data[ACC_Z_IDX];
        // Normalise accelerometer measurement
        recipNorm = q16_mul(ax, ax);
        recipNorm += q16_mul(ay, ay);
        recipNorm += q16_mul(az, az);
        recipNorm = q16_rsqrt(recipNorm);
        ax = q16_mul(ax, recipNorm);
        ay = q16_mul(ay, recipNorm);
        az = q16_mul(az, recipNorm);

        // Auxiliary variables to avoid repeated arithmetic
        _2q1q1q2q2 = q16_mul(q1, q1) + q16_mul(q2, q2);
        tmpterm = az + _2q1q1q2q2;
        tmpterm += tmpterm;
        _2q1q1q2q2 += _2q1q1q2q2;

        // Gradient descent algorithm corrective step
        s0 = q16_mul(q0, _2q1q1q2q2) + q16_mul(q2, ax) - q16_mul(q1, ay);
        s1 = q16_mul(q1, tmpterm) - q16_mul(q3, ax) - q16_mul(q0, ay);
        s2 = q16_mul(q2, tmpterm) + q16_mul(q0, ax) - q16_mul(q3, ay);
        s3 = q16_mul(q3, _2q1q1q2q2) - q16_mul(q1, ax) - q16_mul(q2, ay);

        // Apply feedback step
        recipNorm = q16_mul(s0, s0);
        recipNorm += q16_mul(s1, s1);
        recipNorm += q16_mul(s2, s2);
        recipNorm += q16_mul(s3, s3);
        recipNorm = q16_rsqrt(recipNorm);
        recipNorm = q16_mul(beta, recipNorm);
        qDot0 -= q16_mul(recipNorm, s0);
        qDot1 -= q16_mul(recipNorm, s1);
        qDot2 -= q16_mul(recipNorm, s2);
        qDot3 -= q16_mul(recipNorm, s3);
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += q16_mul(qDot0, gyroIntegrationFactor);
    q1 += q16_mul(qDot1, gyroIntegrationFactor);
    q2 += q16_mul(qDot2, gyroIntegrationFactor);
    q3 += q16_mul(qDot3, gyroIntegrationFactor);

    // Normalise quaternion
    recipNorm = q16_mul(q0, q0);
    recipNorm += q16_mul(q1, q1);
    recipNorm += q16_mul(q2, q2);
    recipNorm += q16_mul(q3, q3);
    recipNorm = q16_rsqrt(recipNorm);
    q->q0 = q16_mul(q0, recipNorm);
    q->q1 = q16_mul(q1, recipNorm);
    q->q2 = q16_mul(q2, recipNorm);
    q->q3 = q16_mul(q3, recipNorm);
}

