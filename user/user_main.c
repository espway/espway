/*
    Firmware for a segway-style robot using ESP8266.
    Copyright (C) 2016  Sakari Kapanen

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

#include <esp8266.h>
#include <driver/uart.h>
#include <pwm.h>

#include "ws2812_i2s.h"

// libesphttpd includes
#include "httpd.h"
#include "cgiflash.h"
#include "httpdespfs.h"
#include "espfs.h"
#include "webpages-espfs.h"
#include "cgiwebsocket.h"

// internal includes
#include "i2c.h"
#include "mpu6050.h"
#include "imu.h"
#include "pid.h"

#include "config.h"

#define QUEUE_LEN 1
#define MPU_ADDR 0x68
#define M_PI 3.14159265359f

#define LOGMODE LOG_PITCH

#define FALL_LOWER_BOUND FLT_TO_Q16(STABLE_ANGLE - FALL_LIMIT)
#define FALL_UPPER_BOUND FLT_TO_Q16(STABLE_ANGLE + FALL_LIMIT)
#define RECOVER_LOWER_BOUND FLT_TO_Q16(STABLE_ANGLE - RECOVER_LIMIT)
#define RECOVER_UPPER_BOUND FLT_TO_Q16(STABLE_ANGLE + RECOVER_LIMIT)

#define BATTERY_COEFFICIENT FLT_TO_Q16(100.0f / BATTERY_CALIBRATION_FACTOR)

#define FREQUENCY_SAMPLES 1000
#define QUAT_DELAY 50

#define MPU_RATE 0
#define SAMPLE_TIME ((1.0f + MPU_RATE) / 1000.0f)

#define PWMPERIOD 2500

const int LED_PIN = 2;

os_event_t gTaskQueue[QUEUE_LEN];

typedef enum { LOG_FREQ, LOG_RAW, LOG_PITCH, LOG_NONE } logmode;
typedef enum { STABILIZING_ORIENTATION, RUNNING, FALLEN } state;

typedef struct {
    uint8_t r, g, b;
} color_t;

const color_t RED = { 180, 0, 0 };
const color_t YELLOW = { 180, 180, 0 };
const color_t GREEN = { 0, 180, 0 };
const color_t BLUE = { 0, 0, 180 };
const color_t LILA = { 180, 0, 180 };
const color_t BLACK = { 0, 0, 0 };

typedef enum {
    STEERING = 0,
    REQ_QUATERNION = 1,
    RES_QUATERNION = 2,
    BATTERY = 3
} ws_msg_type;

madgwickparams imuparams;
pidsettings velPidSettings;
pidsettings anglePidSettings;
pidsettings angleHighPidSettings;
pidstate velPidState;
pidstate anglePidState;

q16 targetSpeed = 0;
q16 steeringBias = 0;

bool mpuInitSucceeded = false;
bool sendQuat = false;
bool otaStarted = false;

int myUploadFirmware(HttpdConnData *connData) {
    otaStarted = true;
    return cgiUploadFirmware(connData);
}

void ICACHE_FLASH_ATTR set_both_eyes(const color_t color) {
    uint8_t buf[] = {
        color.g, color.r, color.b,
        color.g, color.r, color.b
    };
    ws2812_push(buf, 6);
}


void websocketRecvCb(Websock *ws, char *signed_data, int len, int flags) {
    if (len == 0) {
        return;
    }
    uint8_t *data = (uint8_t *)signed_data;
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


void websocketConnectCb(Websock *ws) {
    ws->recvCb = websocketRecvCb;
}


void ICACHE_FLASH_ATTR sendQuaternion(const quaternion_fix * const quat) {
    uint8_t buf[9];
    buf[0] = RES_QUATERNION;
    int16_t *qdata = (int16_t *)&buf[1];
    qdata[0] = quat->q0 / 2;
    qdata[1] = quat->q1 / 2;
    qdata[2] = quat->q2 / 2;
    qdata[3] = quat->q3 / 2;
    cgiWebsockBroadcast("/ws", (char *)buf, 9, WEBSOCK_FLAG_BIN);
    sendQuat = false;
}


void ICACHE_FLASH_ATTR sendBatteryReading(uint16_t batteryReading) {
    uint8_t buf[3];
    buf[0] = BATTERY;
    uint16_t *payload = (uint16_t *)&buf[1];
    payload[0] = q16_mul(batteryReading, BATTERY_COEFFICIENT);
    cgiWebsockBroadcast("/ws", (char *)buf, 3, WEBSOCK_FLAG_BIN);
}

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


void ICACHE_FLASH_ATTR setMotors(int leftSpeed, int rightSpeed) {
    set_motor_speed(1, 12, rightSpeed, REVERSE_RIGHT_MOTOR);
    set_motor_speed(0, 15, leftSpeed, REVERSE_LEFT_MOTOR);
    pwm_start();
}


void ICACHE_FLASH_ATTR doLog(int16_t *rawAccel, int16_t *rawGyro, q16 spitch) {
    static unsigned long lastCallTime = 0;
    static unsigned int callCounter = 0;

    if (LOGMODE == LOG_RAW) {
        os_printf("%d, %d, %d, %d, %d, %d\n",
            rawAccel[0], rawAccel[1], rawAccel[2],
            rawGyro[0], rawGyro[1], rawGyro[2]);
    } else if (LOGMODE == LOG_PITCH) {
        os_printf("%d\n", spitch);
    } else if (LOGMODE == LOG_FREQ) {
        unsigned long us = system_get_time();
        if (++callCounter == FREQUENCY_SAMPLES) {
            os_printf("%lu\n", FREQUENCY_SAMPLES * 1000000 / (us - lastCallTime));
            callCounter = 0;
            lastCallTime = us;
        }
    }
}

void ICACHE_FLASH_ATTR compute(os_event_t *e) {
    if (!mpuInitSucceeded || otaStarted) {
        setMotors(0, 0);
        set_both_eyes(LILA);
        return;
    }

    static unsigned long lastBatteryCheck = 0;
    static unsigned int batteryValue = 1024;
    static bool sendBattery = false;

    unsigned long curTime;

    while (!mpuReadIntStatus(MPU_ADDR)) {
        curTime = system_get_time() / 1024;
        if (ENABLE_BATTERY_CHECK &&
            curTime - lastBatteryCheck > BATTERY_CHECK_INTERVAL) {
            lastBatteryCheck = curTime;
            batteryValue = q16_exponential_smooth(batteryValue, system_adc_read(),
                FLT_TO_Q16(0.25f));
            sendBattery = true;

            if (batteryValue <
                (unsigned int)(BATTERY_THRESHOLD * BATTERY_CALIBRATION_FACTOR)) {
                set_both_eyes(BLACK);
                //mpu.setSleepEnabled(true);
                setMotors(0, 0);
                //ESP.deepSleep(100000000UL);
            }
        }
    }

    // Perform MPU quaternion update
    int16_t rawData[6];
    mpuReadRawData(MPU_ADDR, rawData);
    // Update orientation estimate
    static quaternion_fix quat = { Q16_ONE, 0, 0, 0 };
    MadgwickAHRSupdateIMU_fix(&imuparams, &rawData[0], &rawData[3],
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
    curTime = system_get_time() / 1024;
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
            set_both_eyes(BLUE);
            travelSpeed = 0;
            setMotors(0, 0);
        }
    } else if (myState == FALLEN) {
        if (spitch < RECOVER_UPPER_BOUND && spitch > RECOVER_LOWER_BOUND) {
            myState = RUNNING;
            set_both_eyes(GREEN);
            pid_reset(spitch, 0, &anglePidSettings, &anglePidState);
            pid_reset(0, FLT_TO_Q16(STABLE_ANGLE), &velPidSettings,
                &velPidState);
        }
    }

    if (LOGMODE != LOG_NONE) {
        doLog(&rawData[0], &rawData[3], spitch);
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

    system_os_post(2, 0, 0);
}

void ICACHE_FLASH_ATTR wifi_init(void) {
    struct softap_config config;
    wifi_set_opmode_current(0x02);
    wifi_softap_get_config(&config);
    wifi_set_phy_mode(PHY_MODE_11G);
    os_memset(config.ssid, 0, 32);
    os_memset(config.password, 0, 64);
    os_memcpy(config.ssid, "ESPway", 6);
    config.authmode = AUTH_OPEN;
    config.ssid_len = 0;
    config.beacon_interval = 100;
    config.max_connection = 1;
    wifi_softap_set_config(&config);
}

#ifdef ESPFS_POS
CgiUploadFlashDef uploadParams={
    .type=CGIFLASH_TYPE_ESPFS,
    .fw1Pos=ESPFS_POS,
    .fw2Pos=0,
    .fwSize=ESPFS_SIZE,
};
#define INCLUDE_FLASH_FNS
#endif
#ifdef OTA_FLASH_SIZE_K
CgiUploadFlashDef uploadParams={
    .type=CGIFLASH_TYPE_FW,
    .fw1Pos=0x1000,
    .fw2Pos=((OTA_FLASH_SIZE_K*1024)/2)+0x1000,
    .fwSize=((OTA_FLASH_SIZE_K*1024)/2)-0x1000,
    .tagName=OTA_TAGNAME
};
#define INCLUDE_FLASH_FNS
#endif

HttpdBuiltInUrl builtInUrls[]={
#ifdef INCLUDE_FLASH_FNS
    {"/flash/next", cgiGetFirmwareNext, &uploadParams},
    {"/flash/upload", myUploadFirmware, &uploadParams},
    {"/flash/reboot", cgiRebootFirmware, NULL},
#endif

    {"/ws", cgiWebsocket, websocketConnectCb},
    {"/", cgiRedirect, "/index.html"},
    {"*", cgiEspFsHook, NULL}, //Catch-all cgi function for the filesystem
    {NULL, NULL, NULL}
};

bool ICACHE_FLASH_ATTR mpuInit(void) {
    // Wake up
    mpuWriteRegister(MPU_ADDR, MPU_PWR_MGMT_1, MPU_CLK_PLL_ZGYRO | MPU_TEMP_DIS,
        false);
    // 1000 Hz sampling
    mpuWriteRegister(MPU_ADDR, MPU_SMPRT_DIV, MPU_RATE, false);
    // 188 Hz LPF
    mpuWriteRegister(MPU_ADDR, MPU_CONFIG, 1, false);
    // Gyroscope min sensitivity
    mpuWriteRegister(MPU_ADDR, MPU_GYRO_CONFIG, 3 << 3, false);
    // Accel max sensitivity
    mpuWriteRegister(MPU_ADDR, MPU_ACCEL_CONFIG, 0, false);
    // Data ready interrupt on
    mpuWriteRegister(MPU_ADDR, MPU_INT_ENABLE, 1, false);
    // TODO gyro offset loading
    // Check if the MPU still responds with its own address
    uint8_t addr = 0;
    mpuReadRegisters(MPU_ADDR, MPU_WHO_AM_I, 1, &addr);
    return addr == MPU_ADDR;
}

void ICACHE_FLASH_ATTR user_init(void) {
    system_update_cpu_freq(80);

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

    uart_init(BIT_RATE_115200, BIT_RATE_115200);
    i2c_gpio_init();
    mpuInitSucceeded = mpuInit();

    wifi_init();

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

    ws2812_init();
    set_both_eyes(mpuInitSucceeded ? YELLOW : RED);

    // 0x40200000 is the base address for spi flash memory mapping, ESPFS_POS is the position
    // where image is written in flash that is defined in Makefile.
#ifdef ESPFS_POS
    espFsInit((void*)(0x40200000 + ESPFS_POS));
#else
    espFsInit((void*)(webpages_espfs_start));
#endif
    httpdInit(builtInUrls, 80);

    system_os_task(compute, 2, gTaskQueue, QUEUE_LEN);
    system_os_post(2, 0, 0);
}

