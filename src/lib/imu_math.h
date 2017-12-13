/*
 * Library for calculating orientation from an inertial measurement unit
 * Copyright (C) 2017  Sakari Kapanen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include "q16.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "vector3d.h"

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

#ifdef __cplusplus
}
#endif

