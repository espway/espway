#include "motor.h"

void setupMotorPwm(motor m) {
    pinMode(m.pwmPin, OUTPUT);
    pinMode(m.directionPin, OUTPUT);
    digitalWrite(m.pwmPin, LOW);
    digitalWrite(m.directionPin, LOW);
}

void setMotorVelocity(motor m, double velocity) {
    if (m.reverse) velocity = -velocity;
    int pulsewidth;
    if (velocity < 0.0) {
        digitalWrite(m.directionPin, 1);
        pulsewidth = PWMRANGE * (1.0 - velocity);
    } else {
        digitalWrite(m.directionPin, 0);
        pulsewidth = PWMRANGE * velocity;
    }
    analogWrite(m.pwmPin, pulsewidth);
}

