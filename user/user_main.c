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

#include <user_interface.h>
#include <osapi.h>
#include <espmissingincludes.h>
#include <gpio.h>
#include <driver/uart.h>
#include <stdbool.h>

#include <esp8266.h>
#include "httpd.h"
#include "cgiflash.h"
#include "httpdespfs.h"
#include "espfs.h"
#include "webpages-espfs.h"

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

const int LED_PIN = 2;

os_event_t gTaskQueue[QUEUE_LEN];

typedef enum { LOG_FREQ, LOG_RAW, LOG_PITCH, LOG_NONE } logmode;
typedef enum { STABILIZING_ORIENTATION, RUNNING, FALLEN } state;

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

void setMotors(int x, int y) {}

void doLog(int16_t *rawAccel, int16_t *rawGyro, q16 spitch) {
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
    if (!mpuInitSucceeded) {
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
            /*
            if (batteryValue <
                (unsigned int)(BATTERY_THRESHOLD * BATTERY_CALIBRATION_FACTOR)) {
                //setBothEyes(BLACK);
                //mpu.setSleepEnabled(true);
                setMotors(0, 0);
                //ESP.deepSleep(100000000UL);
            }
            */
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
            //setBothEyes(BLUE);
            travelSpeed = 0;
            setMotors(0, 0);
        }
    } else if (myState == FALLEN) {
        if (spitch < RECOVER_UPPER_BOUND && spitch > RECOVER_LOWER_BOUND) {
            myState = RUNNING;
            //setBothEyes(GREEN);
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
        //sendBatteryReading(batteryValue);
    }

    static unsigned long lastSentQuat = 0;
    if (sendQuat && curTime - lastSentQuat > QUAT_DELAY) {
        //sendQuaternion(&quat);
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
    {"/flash/upload", cgiUploadFirmware, &uploadParams},
    {"/flash/reboot", cgiRebootFirmware, NULL},
#endif

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

