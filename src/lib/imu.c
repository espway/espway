/*
    Library for calculating orientation from an inertial measurement unit
    Copyright (C) 2017  Sakari Kapanen

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

#include "imu.h"

q16 gravity_x(const quaternion_fix * const quat) {
    return 2 * (q16_mul(quat->q1, quat->q3) - q16_mul(quat->q0, quat->q2));
}

q16 gravity_y(const quaternion_fix * const quat) {
    return 2 * (q16_mul(quat->q2, quat->q3) + q16_mul(quat->q0, quat->q1));
}

q16 gravity_z(const quaternion_fix * const quat) {
    return q16_mul(quat->q0, quat->q0) -
        q16_mul(quat->q1, quat->q1) -
        q16_mul(quat->q2, quat->q2) +
        q16_mul(quat->q3, quat->q3);
}

//-----------------------------------------------------------------------------
// IMU algorithm update

//=============================================================================
// originally from MadgwickAHRS.c
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
void madgwick_ahrs_update_imu(
    const madgwickparams * const params,
    const int16_t * const raw_accel, const int16_t * const raw_gyro,
    quaternion_fix * const q) {
    q16 recip_norm;
    q16 q0, q1, q2, q3;
    q16 s0, s1, s2, s3;
    q16 qdot0, qdot1, qdot2, qdot3;
    q16 _2q1q1q2q2, tmpterm;

    q0 = q->q0;
    q1 = q->q1;
    q2 = q->q2;
    q3 = q->q3;

    q16 gx = raw_gyro[0], gy = raw_gyro[1], gz = raw_gyro[2];

    // Rate of change of quaternion from gyroscope
    // NOTE these values are actually double the actual values but it
    // does not matter
    qdot0 = -q16_mul(q1, gx) - q16_mul(q2, gy) - q16_mul(q3, gz);
    qdot1 = q16_mul(q0, gx) + q16_mul(q2, gz) - q16_mul(q3, gy);
    qdot2 = q16_mul(q0, gy) - q16_mul(q1, gz) + q16_mul(q3, gx);
    qdot3 = q16_mul(q0, gz) + q16_mul(q1, gy) - q16_mul(q2, gx);

    q16 ax = raw_accel[0], ay = raw_accel[1], az = raw_accel[2];
    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (ax != 0 || ay != 0 || az != 0) {
        // Normalise accelerometer measurement
        recip_norm = q16_mul(ax, ax);
        recip_norm += q16_mul(ay, ay);
        recip_norm += q16_mul(az, az);
        recip_norm = q16_rsqrt(recip_norm);
        ax = q16_mul(ax, recip_norm);
        ay = q16_mul(ay, recip_norm);
        az = q16_mul(az, recip_norm);

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
        recip_norm = q16_mul(s0, s0);
        recip_norm += q16_mul(s1, s1);
        recip_norm += q16_mul(s2, s2);
        recip_norm += q16_mul(s3, s3);
        recip_norm = q16_rsqrt(recip_norm);
        recip_norm = q16_mul(params->beta, recip_norm);
        qdot0 -= q16_mul(recip_norm, s0);
        qdot1 -= q16_mul(recip_norm, s1);
        qdot2 -= q16_mul(recip_norm, s2);
        qdot3 -= q16_mul(recip_norm, s3);
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += q16_mul(qdot0, params->gyro_integration_factor);
    q1 += q16_mul(qdot1, params->gyro_integration_factor);
    q2 += q16_mul(qdot2, params->gyro_integration_factor);
    q3 += q16_mul(qdot3, params->gyro_integration_factor);

    // Normalise quaternion
    recip_norm = q16_mul(q0, q0);
    recip_norm += q16_mul(q1, q1);
    recip_norm += q16_mul(q2, q2);
    recip_norm += q16_mul(q3, q3);
    recip_norm = q16_rsqrt(recip_norm);
    q->q0 = q16_mul(q0, recip_norm);
    q->q1 = q16_mul(q1, recip_norm);
    q->q2 = q16_mul(q2, recip_norm);
    q->q3 = q16_mul(q3, recip_norm);
}

void calculate_madgwick_params(madgwickparams * const params,
    float beta, float gyro_scale, float sample_time) {
    params->gyro_integration_factor = FLT_TO_Q16(0.5f * gyro_scale * sample_time);
    params->beta = FLT_TO_Q16(beta / (0.5f * gyro_scale));
}

