#include <Wire.h>

#include "mpu6050.h"

const int MPU_ADDR = 0x68;
mpuconfig gMpuConfig = {
    .disableTemp = true,
    .lowpass = 4,
    .sampleRateDivider = 1,
    .gyroRange = 0,
    .accelRange = 3,
    .enableInterrupt = true,
    .intActiveLow = false,
    .intOpenDrain = false,
    .beta = 0.2f
};

volatile bool gDataAvailable = false;
void dataAvailable() {
    gDataAvailable = true;
}

void setup() {
    Serial.begin(115200);

    for (int i = 12; i <= 15; i++) {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);
    }

    Wire.begin(0, 5);
    Wire.setClock(400000L);
    if (mpuSetup(MPU_ADDR, &gMpuConfig) != 0) {
        Serial.println("Setup failed");
        return 0;
    }

    attachInterrupt(4, dataAvailable, RISING);
}

void loop() {
    while (!gDataAvailable) yield();
    gDataAvailable = false;
    mpuReadIntStatus(MPU_ADDR);
}

