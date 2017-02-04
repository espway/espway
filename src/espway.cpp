#include <Arduino.h>
#include <Hash.h>
#include <Wire.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <ESPAsyncWebServer.h>
#include <NeoPixelBus.h>

#include "motor.h"
#include "newpwm.h"
#include "imu.h"
#include "pid.h"

#include "config.h"


enum logmode { LOG_FREQ, LOG_RAW, LOG_PITCH, LOG_NONE };
enum state { STABILIZING_ORIENTATION, RUNNING, FALLEN };

enum ws_msg_type {
    STEERING = 0,
    REQ_QUATERNION = 1,
    RES_QUATERNION = 2,
    BATTERY = 3
};

const logmode LOGMODE = LOG_NONE;

const q16 FALL_LOWER_BOUND = FLT_TO_Q16(STABLE_ANGLE - FALL_LIMIT),
          FALL_UPPER_BOUND = FLT_TO_Q16(STABLE_ANGLE + FALL_LIMIT);
const q16 RECOVER_LOWER_BOUND = FLT_TO_Q16(STABLE_ANGLE - RECOVER_LIMIT),
          RECOVER_UPPER_BOUND = FLT_TO_Q16(STABLE_ANGLE + RECOVER_LIMIT);

const q16 BATTERY_COEFFICIENT = FLT_TO_Q16(100.0f / BATTERY_CALIBRATION_FACTOR);

const int FREQUENCY_SAMPLES = 1000;
const int QUAT_DELAY = 50;

madgwickparams imuparams;
const uint8_t MPU_RATE = 0;
const float SAMPLE_TIME = (1.0f + MPU_RATE) / 1000.0f;
const int MPU_ADDR = 0x68;

pidsettings velPidSettings;
pidsettings anglePidSettings;
pidsettings angleHighPidSettings;
pidstate velPidState;
pidstate anglePidState;
MPU6050 mpu;

q16 targetSpeed = 0;
q16 steeringBias = 0;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> eyes(2);
RgbColor RED(180, 0, 0);
RgbColor YELLOW(180, 180, 0);
RgbColor GREEN(0, 180, 0);
RgbColor BLUE(0, 0, 180);
RgbColor LILA(180, 0, 180);
RgbColor BLACK(0, 0, 0);

bool sendQuat = false;
bool otaStarted = false;
bool mpuInitSucceeded = false;


void setBothEyes(RgbColor &color) {
    eyes.SetPixelColor(0, color);
    eyes.SetPixelColor(1, color);
    eyes.Show();
}


void setMotors(q16 leftSpeed, q16 rightSpeed) {
    setMotorSpeed(1, 12, rightSpeed, REVERSE_RIGHT_MOTOR);
    setMotorSpeed(0, 15, leftSpeed, REVERSE_LEFT_MOTOR);
    pwm_start();
}


void wsCallback(AsyncWebSocket * server, AsyncWebSocketClient * client,
    AwsEventType type, void * arg, uint8_t *data, size_t len) {
    if (len == 0) {
        return;
    }
    uint8_t msgtype = data[0];
    uint8_t *payload = &data[1];
    int data_len = len - 1;
    // Parse steering command
    if (msgtype == STEERING && data_len == 2) {
        int8_t *signed_data = (int8_t *)payload;
        steeringBias = (FLT_TO_Q16(STEERING_FACTOR) * signed_data[0]) / 128;
        targetSpeed = (FLT_TO_Q16(SPEED_CONTROL_FACTOR) * signed_data[1]) / 128;
    } else if (msgtype == REQ_QUATERNION) {
        sendQuat = true;
    }
}


bool mpuInit() {
    noInterrupts();
    mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu.setSleepEnabled(false);
    mpu.setRate(MPU_RATE);
    mpu.setTempSensorEnabled(false);
    mpu.setDLPFMode(MPU6050_DLPF_BW_188);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setIntDataReadyEnabled(true);
    mpu.setIntEnabled(true);
    mpu.setXGyroOffset(GYRO_OFFSETS[0]);
    mpu.setYGyroOffset(GYRO_OFFSETS[1]);
    mpu.setZGyroOffset(GYRO_OFFSETS[2]);
    int id = mpu.getDeviceID();
    interrupts();
    return id == 0x34;
}


// Adapted from I2Cdevlib: https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp,
// adding a finite timeout
int getMotion6(int16_t *accel, int16_t *gyro) {
    uint8_t buffer[14];
    int count = I2Cdev::readBytes(MPU_ADDR, MPU6050_RA_ACCEL_XOUT_H, 14,
        buffer, 2);
    accel[0] = (((int16_t)buffer[0]) << 8) | buffer[1];
    accel[1] = (((int16_t)buffer[2]) << 8) | buffer[3];
    accel[2] = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyro[0] = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyro[1] = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyro[2] = (((int16_t)buffer[12]) << 8) | buffer[13];
    return count;
}


// Adapted from I2Cdevlib: https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp,
// adding a finite timeout
bool getIntDataReadyStatus() {
    uint8_t buffer;
    I2Cdev::readBit(MPU_ADDR, MPU6050_RA_INT_STATUS,
        MPU6050_INTERRUPT_DATA_RDY_BIT, &buffer, 2);
    return buffer;
}


void setup() {
    // Parameter calculation & initialization
    pid_initialize_flt(ANGLE_KP, ANGLE_KI, ANGLE_KD, SAMPLE_TIME,
        -Q16_ONE, Q16_ONE, false, &anglePidSettings);
    pid_initialize_flt(ANGLE_HIGH_KP, ANGLE_HIGH_KI, ANGLE_HIGH_KD, SAMPLE_TIME,
        -Q16_ONE, Q16_ONE, false, &angleHighPidSettings);
    pid_reset(0, 0, &anglePidSettings, &anglePidState);
    pid_initialize_flt(VEL_KP, VEL_KI, VEL_KD, SAMPLE_TIME,
        FALL_LOWER_BOUND, FALL_UPPER_BOUND, true, &velPidSettings);
    pid_reset(0, 0, &velPidSettings, &velPidState);
    calculateMadgwickParams(&imuparams, MADGWICK_BETA,
        2.0f * M_PI / 180.0f * 2000.0f, SAMPLE_TIME);

    // I2C & MPU6050 init
    Wire.begin(4, 5);
    Wire.setClock(400000);
    mpuInitSucceeded = mpuInit();

    if (LOGMODE != LOG_NONE) {
        Serial.begin(115200);
    }

    // NeoPixel init
    eyes.Begin();
    setBothEyes(mpuInitSucceeded ? YELLOW : RED);

    // WiFi soft AP init
    WiFi.persistent(false);
    WiFi.softAP("ESPway", NULL, 1, 0, 1);  // Use this as soon as new Arduino framework is released

    // ESPAsyncWebServer init
    SPIFFS.begin();
    ws.onEvent(wsCallback);
    server.addHandler(&ws);
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
    server.onNotFound([](AsyncWebServerRequest *req) {
        req->send(404);
    });
    server.begin();

    // ArduinoOTA init
    ArduinoOTA.onStart([]() {
        otaStarted = true;
        setMotors(0, 0);
        setBothEyes(LILA);
    });
    ArduinoOTA.begin();

    // GPIO setup
    pinMode(A0, INPUT);
    pinMode(12, OUTPUT);
    pinMode(15, OUTPUT);
    digitalWrite(12, LOW);
    digitalWrite(15, LOW);
    // PWM init
    pwm_add_channel(13);
    pwm_add_channel(14);
    pwm_init();
}


void doLog(int16_t *rawAccel, int16_t *rawGyro, q16 spitch) {
    static unsigned long lastCallTime = 0;
    static int callCounter = 0;

    if (LOGMODE == LOG_RAW) {
        Serial.print(rawAccel[0]); Serial.print(',');
        Serial.print(rawAccel[1]); Serial.print(',');
        Serial.print(rawAccel[2]); Serial.print(',');
        Serial.print(rawGyro[0]); Serial.print(',');
        Serial.print(rawGyro[1]); Serial.print(',');
        Serial.println(rawGyro[2]);
    } else if (LOGMODE == LOG_PITCH) {
        Serial.println(spitch);
    } else if (LOGMODE == LOG_FREQ) {
        unsigned long ms = millis();
        if (++callCounter == FREQUENCY_SAMPLES) {
            Serial.println(FREQUENCY_SAMPLES * 1000 / (ms - lastCallTime));
            callCounter = 0;
            lastCallTime = ms;
        }
    }
}


void sendQuaternion(const quaternion_fix * const quat) {
    uint8_t buf[9];
    buf[0] = RES_QUATERNION;
    int16_t *qdata = (int16_t *)&buf[1];
    qdata[0] = quat->q0 / 2;
    qdata[1] = quat->q1 / 2;
    qdata[2] = quat->q2 / 2;
    qdata[3] = quat->q3 / 2;
    ws.binaryAll(buf, 9);
    sendQuat = false;
}

void sendBatteryReading(uint16_t batteryReading) {
    uint8_t buf[3];
    buf[0] = BATTERY;
    uint16_t *payload = (uint16_t *)&buf[1];
    payload[0] = q16_mul(batteryReading, BATTERY_COEFFICIENT);
    ws.binaryAll(buf, 3);
}


void loop() {
    if (otaStarted) {
        ArduinoOTA.handle();
        return;
    }

    if (!mpuInitSucceeded) {
        return;
    }

    static unsigned long lastBatteryCheck = 0;
    static unsigned int batteryValue = 1024;
    static bool sendBattery = false;

    unsigned long curTime;

    while (!getIntDataReadyStatus()) {
        curTime = millis();
        if (ENABLE_BATTERY_CHECK &&
            curTime - lastBatteryCheck > BATTERY_CHECK_INTERVAL) {
            lastBatteryCheck = curTime;
            batteryValue = q16_exponential_smooth(batteryValue, analogRead(A0),
                FLT_TO_Q16(0.25f));
            sendBattery = true;
            if (batteryValue <
                (unsigned int)(BATTERY_THRESHOLD * BATTERY_CALIBRATION_FACTOR)) {
                setBothEyes(BLACK);
                mpu.setSleepEnabled(true);
                setMotors(0, 0);
                ESP.deepSleep(100000000UL);
            }
        }
    }

    // Perform MPU quaternion update
    int16_t rawAccel[3];
    int16_t rawGyro[3];
    getMotion6(rawAccel, rawGyro);
    // Update orientation estimate
    static quaternion_fix quat = { Q16_ONE, 0, 0, 0 };
    MadgwickAHRSupdateIMU_fix(&imuparams, rawAccel, rawGyro,
        &quat);
    // Calculate sine of pitch angle from quaternion
    q16 spitch = -gravityZ(&quat);

    static q16 travelSpeed = 0;
    static q16 smoothedTargetSpeed = 0;

    // Exponential smoothing of target speed
    smoothedTargetSpeed = q16_exponential_smooth(smoothedTargetSpeed,
        targetSpeed, FLT_TO_Q16(TARGET_SPEED_SMOOTHING));

    static state myState = STABILIZING_ORIENTATION;
    static unsigned long stageStarted = 0;
    curTime = millis();
    if (myState == STABILIZING_ORIENTATION) {
        if (curTime - stageStarted > ORIENTATION_STABILIZE_DURATION) {
            myState = RUNNING;
            stageStarted = curTime;
        }
    } else if (myState == RUNNING) {
        if (spitch < FALL_UPPER_BOUND && spitch > FALL_LOWER_BOUND) {
            // Perform PID update
            q16 targetAngle = pid_compute(travelSpeed, smoothedTargetSpeed,
                &velPidSettings, &velPidState);
            bool useHighPid =
                spitch < (targetAngle - FLT_TO_Q16(HIGH_PID_LIMIT)) ||
                spitch > (targetAngle + FLT_TO_Q16(HIGH_PID_LIMIT));
            q16 motorSpeed = pid_compute(spitch, targetAngle,
                useHighPid ? &angleHighPidSettings : &anglePidSettings,
                &anglePidState);

            setMotors(motorSpeed + steeringBias, motorSpeed - steeringBias);

            // Estimate travel speed by exponential smoothing
            travelSpeed = q16_exponential_smooth(travelSpeed, motorSpeed,
                FLT_TO_Q16(TRAVEL_SPEED_SMOOTHING));
        } else {
            myState = FALLEN;
            setBothEyes(BLUE);
            travelSpeed = 0;
            setMotors(0, 0);
        }
    } else if (myState == FALLEN) {
        if (spitch < RECOVER_UPPER_BOUND && spitch > RECOVER_LOWER_BOUND) {
            myState = RUNNING;
            setBothEyes(GREEN);
            pid_reset(spitch, 0, &anglePidSettings, &anglePidState);
            pid_reset(0, FLT_TO_Q16(STABLE_ANGLE), &velPidSettings,
                &velPidState);
        }
    }

    if (LOGMODE != LOG_NONE) {
        doLog(rawAccel, rawGyro, spitch);
    }

    if (sendBattery) {
        sendBattery = false;
        sendBatteryReading(batteryValue);
    }

    static unsigned long lastSentQuat = 0;
    if (sendQuat && curTime - lastSentQuat > QUAT_DELAY) {
        sendQuaternion(&quat);
        lastSentQuat = curTime;
    }

    static unsigned long lastOtaHandled = 0;
    if (curTime - lastOtaHandled > 500) {
        ArduinoOTA.handle();
        lastOtaHandled = curTime;
    }
}

