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

#pragma once

#define ACC_X_IDX  0
#define ACC_Y_IDX  1
#define ACC_Z_IDX  2
#define GYRO_X_IDX 3
#define GYRO_Y_IDX 4
#define GYRO_Z_IDX 5

#include <stdint.h>
#include "q16.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "vector3d.h"

// Complementary filter

typedef struct {
    q16 alpha;  // The complementary filter parameter
    q16 gyro_conversion_factor;  // Convert gyro reading from LSB units to rad / us
} complementary_filter_params;

void complementary_filter_update(const complementary_filter_params *params,
    const int dt, const int16_t *raw_accel, const int16_t *raw_gyro,
    vector3d_fix *gravity);

// Mahony filter. Pretty similar to the complementary filter

typedef struct {
    q16 Kp;
    q16 Ki;
    q16 dt;
    vector3d_fix integral;
    q16 gyro_conversion_factor;
} mahony_filter_state;

void mahony_filter_init(mahony_filter_state *state, float Kp, float Ki,
    float gyro_factor, float dt);

void mahony_filter_update(mahony_filter_state *params,
    const int16_t *raw_accel, const int16_t *raw_gyro, vector3d_fix *gravity);

//=====================================================================================================
// originally from MadgwickAHRS.h
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
typedef struct {
    q16 beta;
    q16 gyro_integration_factor;
} madgwickparams;

typedef struct {
    q16 q0, q1, q2, q3;
} quaternion_fix;

q16 gravity_x(const quaternion_fix * const quat);
q16 gravity_y(const quaternion_fix * const quat);
q16 gravity_z(const quaternion_fix * const quat);

void madgwick_ahrs_update_imu(const madgwickparams * const params,
    const int16_t * const raw_accel, const int16_t * const raw_gyro,
    quaternion_fix * const q);

void calculate_madgwick_params(madgwickparams * const params,
    float beta, float gyro_scale, float sample_time);

#ifdef __cplusplus
}
#endif

