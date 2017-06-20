#pragma once

#include <stdint.h>
#include <stddef.h>

#define PWM_MAX_CHANNELS 2

extern int32_t PWMPERIOD;

int pwm_add_channel(uint8_t pin);
void pwm_init(uint32_t period = 2500);

extern "C" {
    void pwm_start(void);
    void pwm_set_duty(uint32_t duty, uint8_t channel);
    void pwm_init(uint32_t period, uint32_t *duty, uint32_t pwm_channel_num,
        uint32_t (*pin_info_list)[3]);
}
