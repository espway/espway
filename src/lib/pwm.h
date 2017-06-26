#pragma once

#include <stdint.h>
#include <stddef.h>

#define PWM_MAX_CHANNELS 2

extern uint32_t pwm_period;

void pwm_start(void);
void pwm_set_duty(uint32_t duty, uint8_t channel);
void pwm_init(uint32_t period, uint32_t *duty, uint32_t pwm_channel_num,
    uint8_t gpio_pins[]);
void pwm_set_period(uint32_t period);

