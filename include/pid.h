/*
    A simple fixed-point PID library
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

#include "q16.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    q16 Kp;
    q16 Ki_times_dt;
    q16 Kd_over_dt;
    q16 dt;
    q16 out_min;
    q16 out_max;
    bool invert;
} pidsettings;

typedef struct {
    q16 i_term;
    q16 last_input;
} pidstate;

void pid_initialize(q16 Kp, q16 Ki, q16 Kd, q16 dt, q16 out_min, q16 out_max,
    bool invert, pidsettings *settings);
void pid_update_params(q16 Kp, q16 Ki, q16 Kd, pidsettings *settings);
void pid_initialize_flt(float Kp, float Ki, float Kd, float dt, q16 out_min,
    q16 out_max, bool invert, pidsettings *settings);
q16 pid_compute(q16 input, q16 setpoint,
    pidsettings *settings, pidstate *state);
void pid_reset(q16 input, q16 output, pidsettings *settings, pidstate *state);

#ifdef __cplusplus
}
#endif

