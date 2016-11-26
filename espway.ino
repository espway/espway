#include <Wire.h>

#include "mpu6050.h"

const int MPU_ADDR = 0x68;
mpuconfig gMpuConfig = {
    .lowpass = 3,
    .sampleRateDivider = 9,
    .gyroRange = 3,
    .accelRange = 0,
    .enableInterrupt = true,
    .intActiveLow = false,
    .intOpenDrain = false,
    .beta = 0.05f
};

volatile float roll, pitch;

const bool TRACK_UPDATE_FREQUENCY = false;
unsigned int gNumCalculations = 0;
unsigned long gLastReportTime = 0;

volatile bool gDataAvailable = false;
void dataAvailable() {
    gDataAvailable = true;
}

void setup() {
    Serial.begin(115200);

#if defined(ESP8266)
    for (int i = 12; i <= 15; ++i) {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);
    }

    Wire.begin(0, 5);
#else
    Wire.begin();
#endif
    Wire.setClock(400000L);
    if (mpuSetup(MPU_ADDR, &gMpuConfig) != 0) {
        Serial.println("Setup failed");
        return;
    }

#if defined(ESP8266)
    attachInterrupt(4, dataAvailable, RISING);
#else
    attachInterrupt(digitalPinToInterrupt(2), dataAvailable, RISING);
#endif
}

void loop() {
    while (!gDataAvailable) yield();
    gDataAvailable = false;

    int16_t buf[6];
    mpuReadIntStatus(MPU_ADDR);
    if (mpuReadRawData(MPU_ADDR, buf) != 0) return;
    mpuUpdateQuaternion(&gMpuConfig, buf);

    roll = rollAngleTaylor();
    pitch = pitchAngleTaylor();

    if (TRACK_UPDATE_FREQUENCY) {
        if (++gNumCalculations == 1000) {
            gNumCalculations = 0;
            unsigned long time = millis();
            Serial.println(1000000L / (time - gLastReportTime));
            gLastReportTime = time;
        }
    } else {
        Serial.print(100.0f * roll);
        Serial.print(',');
        Serial.println(100.0f * pitch);
    }
}

