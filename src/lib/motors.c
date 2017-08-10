/*
 * Firmware for a segway-style robot using ESP8266.
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

#include "motors.h"
#include "pwm.h"
#include "esp/gpio.h"

void set_motor_speed(int channel, int dir_pin, q16 speed, bool reverse) {
  int32_t period = pwm_period;
  speed = Q16_TO_INT(period * speed);

  if (speed > period) {
    speed = period;
  } else if (speed < -period) {
    speed = -period;
  }

  if (reverse) {
    speed = -speed;
  }

  if (speed < 0) {
    gpio_write(dir_pin, 1);
    pwm_set_duty(period + speed, channel);
  } else {
    gpio_write(dir_pin, 0);
    pwm_set_duty(speed, channel);
  }
}

void set_motors(q16 left_speed, q16 right_speed) {
  set_motor_speed(1, 12, right_speed, REVERSE_RIGHT_MOTOR);
  set_motor_speed(0, 15, left_speed, REVERSE_LEFT_MOTOR);
  pwm_start();
}

void motors_init(int period) {
  // Motor direction pins
  gpio_enable(12, GPIO_OUTPUT);
  gpio_enable(15, GPIO_OUTPUT);
  gpio_write(12, 0);
  gpio_write(15, 0);
  // PWM init
  uint32_t duty[] = { 0, 0 };
  uint8_t pins[] = { 13, 14 };
  pwm_init(period, duty, 2, pins);
}

