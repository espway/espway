/*
 * Firmware for a segway-style robot using ESP8266.
 * Copyright (C) 2017  Sakari Kapanen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "motors.h"
#include "delta_sigma.h"
#include "esp/gpio.h"
#include "espway_config.h"

#define DS_PERIOD 64
#define DS_RANGE  32

void set_motor_speed(int channel, int dir_pin, q16 speed, bool reverse)
{
  static const int32_t period = DS_RANGE;
  speed = Q16_TO_INT(period * speed);

  if (speed > period) speed = period;
  else if (speed < -period) speed = -period;
  if (reverse) speed = -speed;

#if MOTOR_DRIVER == MOTOR_DRIVER_L293D

  if (speed < 0)
  {
    gpio_write(dir_pin, 1);
    delta_sigma_set_duty(channel, period + speed);
  }
  else
  {
    gpio_write(dir_pin, 0);
    delta_sigma_set_duty(channel, speed);
  }
#elif MOTOR_DRIVER == MOTOR_DRIVER_DRV8835
  gpio_write(dir_pin, speed < 0);
  delta_sigma_set_duty(channel, speed < 0 ? -speed : speed);
#endif
}

void set_motors(q16 left_speed, q16 right_speed)
{
  set_motor_speed(0, MOTOR_RIGHT_DIR_PIN, right_speed, REVERSE_RIGHT_MOTOR);
  set_motor_speed(1, MOTOR_LEFT_DIR_PIN, left_speed, REVERSE_LEFT_MOTOR);
}

void motors_init(void)
{
  // Motor direction pins
  gpio_enable(MOTOR_LEFT_DIR_PIN, GPIO_OUTPUT);
  gpio_enable(MOTOR_RIGHT_DIR_PIN, GPIO_OUTPUT);
  gpio_write(MOTOR_LEFT_DIR_PIN, 0);
  gpio_write(MOTOR_RIGHT_DIR_PIN, 0);
  uint8_t pins[] = {MOTOR_RIGHT_PWM_PIN, MOTOR_LEFT_PWM_PIN};
  delta_sigma_start(DS_PERIOD, DS_RANGE, pins, 2);
}

