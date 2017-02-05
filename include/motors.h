#pragma once

#include "q16.h"

#define REVERSE_RIGHT_MOTOR true
#define REVERSE_LEFT_MOTOR true

#define PWMPERIOD 2500

void set_motor_speed(int channel, int dir_pin, q16 speed, bool reverse);
void set_motors(q16 left_speed, q16 right_speed);
void motors_init(void);

