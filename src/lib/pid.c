/*
 * A simple fixed-point PID library
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

#include "pid.h"

static inline q16 clamp(q16 x, q16 a, q16 b) {
  return x > a ? (x < b ? x : b) : a;
}

void pid_initialize(const pid_coeffs * coeffs, q16 dt,
    q16 out_min, q16 out_max, bool invert, pidsettings *settings) {
  settings->dt = dt;
  settings->out_min = out_min;
  settings->out_max = out_max;
  settings->invert = invert;
  pid_update_params(coeffs, settings);
}

void pid_update_params(const pid_coeffs * coeffs,
    pidsettings *settings) {
  settings->Kp = coeffs->p;
  settings->Ki_times_dt = q16_mul(coeffs->i, settings->dt);
  settings->Kd_over_dt = q16_div(coeffs->d, settings->dt);
  if (settings->invert) {
    settings->Kp *= -1;
    settings->Ki_times_dt *= -1;
    settings->Kd_over_dt *= -1;
  }
}

q16 pid_compute(q16 input, q16 setpoint,
    pidsettings *settings, pidstate *state) {
  q16 error = input - setpoint;
  q16 p_term = q16_mul(settings->Kp, error);
  state->i_term = clamp(state->i_term + q16_mul(settings->Ki_times_dt, error),
      settings->out_min, settings->out_max);
  q16 d_term = q16_mul(settings->Kd_over_dt, input - state->last_input);
  q16 output = p_term + state->i_term + d_term;
  state->last_input = input;
  return clamp(output, settings->out_min, settings->out_max);
}

void pid_reset(q16 input, q16 output, pidsettings *settings, pidstate *state) {
  state->i_term = clamp(output, settings->out_min,
      settings->out_max);
  state->last_input = input;
}

