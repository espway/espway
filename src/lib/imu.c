/*
 * Library for calculating orientation from an inertial measurement unit
 * Copyright (C) 2017  Sakari Kapanen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "imu.h"

void mahony_filter_init(mahony_filter_state *state, float Kp, float Ki,
    float gyro_factor, float dt) {
  state->Kp = FLT_TO_Q16(Kp);
  state->Ki = FLT_TO_Q16(Ki);
  state->dt = FLT_TO_Q16(dt);
  state->gyro_conversion_factor = FLT_TO_Q16(gyro_factor);
  state->integral.x = 0;
  state->integral.y = 0;
  state->integral.z = 0;
}

void mahony_filter_update(mahony_filter_state *state,
    const int16_t *raw_accel, const int16_t *raw_gyro, vector3d_fix *gravity) {
  vector3d_fix omega, accel, verror;
  accel.x = raw_accel[0];
  accel.y = raw_accel[1];
  accel.z = raw_accel[2];
  omega.x = raw_gyro[0];
  omega.y = raw_gyro[1];
  omega.z = raw_gyro[2];

  omega = v3d_mul(state->gyro_conversion_factor, &omega);

  if (accel.x != 0 || accel.y != 0 || accel.z != 0) {
    accel = v3d_normalize(&accel);
    verror = v3d_cross(&accel, gravity);
    state->integral = v3d_add(&state->integral, &verror);
    verror = v3d_mul(state->Kp, &verror);
    omega = v3d_add(&omega, &verror);
  }

  verror = v3d_mul(state->Ki, &state->integral);
  verror = v3d_mul(state->dt, &verror);
  omega = v3d_add(&omega, &verror);

  vector3d_fix vupdate = v3d_cross(gravity, &omega);
  vupdate = v3d_mul(state->dt, &vupdate);
  *gravity = v3d_add(gravity, &vupdate);
  *gravity = v3d_normalize(gravity);
}
