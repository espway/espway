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

