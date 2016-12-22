#include "motor.h"

#include "ets_sys.h"
#include "espmissingincludes.h"
#include "pwm.h"
#include "gpio.h"

static const uint32_t duty_max = MOTOR_PWM_PERIOD * 1000 / 45;

void ICACHE_FLASH_ATTR setup_motor_pwm() {
    ETS_GPIO_INTR_DISABLE();

    uint32_t pin_info_list[][3] = {
        { MOTOR1_PWM_MUX, MOTOR1_PWM_FUNC, MOTOR1_PWM_GPIO_NUM },
        { MOTOR2_PWM_MUX, MOTOR2_PWM_FUNC, MOTOR2_PWM_GPIO_NUM }
    };
    uint32_t duty[] = { 0, 0 };
    pwm_init(MOTOR_PWM_PERIOD, duty, 2, pin_info_list);

    PIN_FUNC_SELECT(MOTOR1_DIR_MUX, MOTOR1_DIR_FUNC);
    PIN_FUNC_SELECT(MOTOR2_DIR_MUX, MOTOR2_DIR_FUNC);
    uint32_t mask = (1 << MOTOR1_DIR_GPIO_NUM) | (1 << MOTOR2_DIR_GPIO_NUM);
    gpio_output_set(0, mask, mask, 0);

    ETS_GPIO_INTR_ENABLE();
}

static void ICACHE_FLASH_ATTR set_motor_velocity(q16 velocity,
    uint8_t pwm_channel, uint8_t gpio_num) {
    int16_t duty = q16_to_int(q16_mul(int_to_q16(duty_max), velocity));
    if (velocity < 0) {
        duty = duty_max - duty;
        GPIO_OUTPUT_SET(gpio_num, 1);
    } else {
        GPIO_OUTPUT_SET(gpio_num, 0);
    }
    if (duty < 0) duty = 0;
    else if (duty > duty_max) duty = duty_max;
    pwm_set_duty(pwm_channel, duty);
}

void ICACHE_FLASH_ATTR set_motor_velocities(q16 velocity1, q16 velocity2) {
    set_motor_velocity(MOTOR1_REVERSE ? -velocity1 : velocity1,
        0, MOTOR1_DIR_GPIO_NUM);
    set_motor_velocity(MOTOR2_REVERSE ? -velocity2 : velocity2,
        0, MOTOR2_DIR_GPIO_NUM);
}

