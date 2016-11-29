#pragma once

const unsigned long MOTOR_PWM_PERIOD = 1000;  // period in us

struct motor {
    int pwmPin;
    int directionPin;
    bool reverse;
};

void setupMotorPwm(motor m);
void setMotorVelocity(motor m, double velocity);

