#pragma once

typedef struct {
    float Kp;
    float Ki;
    float Ki_times_dt;
    float Kd;
    float Kd_over_dt;
    float dt;
    float out_min;
    float out_max;
} pidsettings;

typedef struct {
    float i_term;
    float last_error;
} pidstate;

void pid_initialize(float Kp, float Ki, float Kd, float dt, float out_min,
    float out_max, pidsettings *settings, pidstate *state);
float pid_compute(float input, float setpoint,
    pidsettings *settings, pidstate *state);
void pid_reset(float input, float setpoint, float output,
    pidsettings *settings, pidstate *state);

