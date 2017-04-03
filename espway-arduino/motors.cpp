/*
    Firmware for a segway-style robot using ESP8266.
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

#include <Arduino.h>

#include "motors.h"
#include "newpwm.h"

void set_motor_speed(int channel, int dir_pin, q16 speed, bool reverse) {
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
        digitalWrite(dir_pin, HIGH);
        pwm_set_duty(PWMPERIOD + speed, channel);
    } else {
        digitalWrite(dir_pin, LOW);
        pwm_set_duty(speed, channel);
    }
}

void set_motors(q16 left_speed, q16 right_speed) {
    set_motor_speed(1, 12, right_speed, REVERSE_RIGHT_MOTOR);
    set_motor_speed(0, 15, left_speed, REVERSE_LEFT_MOTOR);
    pwm_start();
}

void motors_init(void) {
    // Motor direction pins
    pinMode(12, OUTPUT);
    pinMode(15, OUTPUT);
    digitalWrite(12, LOW);
    digitalWrite(15, LOW);
        // PWM init
    pwm_add_channel(13);
    pwm_add_channel(14);
    pwm_init();
}

