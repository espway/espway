#include <esp8266.h>
#include <pwm.h>

#include "motors.h"

void ICACHE_FLASH_ATTR set_motor_speed(int channel, int dir_pin, q16 speed,
    bool reverse) {
    speed = Q16_TO_INT(PWMPERIOD * speed);

    if (speed > PWMPERIOD) {
        speed = PWMPERIOD;
    } else if (speed < -PWMPERIOD) {
        speed = -PWMPERIOD;
    }

    if (reverse) {
        speed = -speed;
    }

    if (speed < 0) {
        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1 << dir_pin);
        pwm_set_duty(PWMPERIOD + speed, channel);
    } else {
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << dir_pin);
        pwm_set_duty(speed, channel);
    }
}

void ICACHE_FLASH_ATTR set_motors(q16 left_speed, q16 right_speed) {
    set_motor_speed(1, 12, right_speed, REVERSE_RIGHT_MOTOR);
    set_motor_speed(0, 15, left_speed, REVERSE_LEFT_MOTOR);
    pwm_start();
}

void ICACHE_FLASH_ATTR motors_init(void) {
    // Motor direction pins
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);
    GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, (1 << 12) | (1 << 15));
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 12) | (1 << 15));
    uint32_t pin_info_list[][3] = {
        { PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13, 13 },
        { PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14, 14 }
    };
    uint32_t duty_init[] = { 0, 0 };
    pwm_init(PWMPERIOD, duty_init, 2, pin_info_list);
}

