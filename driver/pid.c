#include "ets_sys.h"
#include "pid.h"

static inline float clamp(float x, float a, float b) {
    return x > a ? (x < b ? x : b) : a;
}

void ICACHE_FLASH_ATTR pid_initialize(float Kp, float Ki, float Kd, float dt,
    float out_min, float out_max, pidsettings *settings, pidstate *state) {
    settings->Kp = Kp;
    settings->Ki = Ki;
    settings->Ki_times_dt = Ki * dt;
    settings->Kd = Kd;
    settings->Kd_over_dt = Kd / dt;
    settings->dt = dt;
    settings->out_min = out_min;
    settings->out_max = out_max;
}

float ICACHE_FLASH_ATTR pid_compute(float input, float setpoint,
    pidsettings *settings, pidstate *state) {
    float error = input - setpoint;
    float p_term = settings->Kp * error;
    state->i_term = clamp(state->i_term + settings->Ki_times_dt * error,
        settings->out_min, settings->out_max);
    float d_term = settings->Kd_over_dt * (error - state->last_error);
    float output = p_term + state->i_term + d_term;
    state->last_error = error;
    return clamp(output, settings->out_min, settings->out_max);
}

void ICACHE_FLASH_ATTR pid_reset(float input, float setpoint, float output,
    pidsettings *settings, pidstate *state) {
    state->i_term = clamp(output, settings->out_min,
        settings->out_max);
    state->last_error = input - setpoint;
}

