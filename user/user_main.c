/*
    Firmware for a segway-style robot using ESP8266.
    Copyright (C) 201&  Sakari Kapanen

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
#include "motors.h"
#include "eyes.h"

#include "config.h"

#define QUEUE_LEN 1
#define M_PI 3.14159265359f

#define LOGMODE LOG_PITCH

#define FALL_LOWER_BOUND FLT_TO_Q16(STABLE_ANGLE - FALL_LIMIT)
#define FALL_UPPER_BOUND FLT_TO_Q16(STABLE_ANGLE + FALL_LIMIT)
#define RECOVER_LOWER_BOUND FLT_TO_Q16(STABLE_ANGLE - RECOVER_LIMIT)
#define RECOVER_UPPER_BOUND FLT_TO_Q16(STABLE_ANGLE + RECOVER_LIMIT)

#define BATTERY_COEFFICIENT FLT_TO_Q16(100.0f / BATTERY_CALIBRATION_FACTOR)

#define FREQUENCY_SAMPLES 1000
#define QUAT_DELAY 50

#define SAMPLE_TIME ((1.0f + MPU_RATE) / 1000.0f)

const color_t RED = { 180, 0, 0 };
const color_t YELLOW = { 180, 180, 0 };
const color_t GREEN = { 0, 180, 0 };
const color_t BLUE = { 0, 0, 180 };
const color_t LILA = { 180, 0, 180 };
const color_t BLACK = { 0, 0, 0 };

os_event_t task_queue[QUEUE_LEN];

typedef enum { LOG_FREQ, LOG_RAW, LOG_PITCH, LOG_NONE } logmode;
typedef enum { STABILIZING_ORIENTATION, RUNNING, FALLEN } state;

typedef enum {
    STEERING = 0,
    REQ_QUATERNION = 1,
    RES_QUATERNION = 2,
    BATTERY = 3
} ws_msg_type;

madgwickparams imuparams;
pidsettings vel_pid_settings;
pidsettings angle_pid_settings;
pidsettings angle_high_pid_settings;
pidstate vel_pid_state;
pidstate angle_pid_state;

q16 target_speed = 0;
q16 steering_bias = 0;

bool mpu_init_succeeded = false;
bool send_quat = false;
bool ota_started = false;

int upload_firmware(HttpdConnData *connData) {
    ota_started = true;
    return cgiUploadFirmware(connData);
}

void websocket_recv_cb(Websock *ws, char *signed_data, int len, int flags) {
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
        steering_bias = (FLT_TO_Q16(STEERING_FACTOR) * signed_data[0]) / 128;
        target_speed = (FLT_TO_Q16(SPEED_CONTROL_FACTOR) * signed_data[1]) / 128;
    } else if (msgtype == REQ_QUATERNION) {
        send_quat = true;
    }
}


void websocket_connect_cb(Websock *ws) {
    ws->recvCb = websocket_recv_cb;
}


void ICACHE_FLASH_ATTR send_quaternion(const quaternion_fix * const quat) {
    uint8_t buf[9];
    buf[0] = RES_QUATERNION;
    int16_t *qdata = (int16_t *)&buf[1];
    qdata[0] = quat->q0 / 2;
    qdata[1] = quat->q1 / 2;
    qdata[2] = quat->q2 / 2;
    qdata[3] = quat->q3 / 2;
    cgiWebsockBroadcast("/ws", (char *)buf, 9, WEBSOCK_FLAG_BIN);
    send_quat = false;
}


void ICACHE_FLASH_ATTR send_battery_reading(uint16_t battery_reading) {
    uint8_t buf[3];
    buf[0] = BATTERY;
    uint16_t *payload = (uint16_t *)&buf[1];
    payload[0] = q16_mul(battery_reading, BATTERY_COEFFICIENT);
    cgiWebsockBroadcast("/ws", (char *)buf, 3, WEBSOCK_FLAG_BIN);
}



void ICACHE_FLASH_ATTR do_log(int16_t *raw_accel, int16_t *raw_gyro,
    q16 sin_pitch) {
    static unsigned long last_call_time = 0;
    static unsigned int call_counter = 0;

    if (LOGMODE == LOG_RAW) {
        os_printf("%d, %d, %d, %d, %d, %d\n",
            raw_accel[0], raw_accel[1], raw_accel[2],
            raw_gyro[0], raw_gyro[1], raw_gyro[2]);
    } else if (LOGMODE == LOG_PITCH) {
        os_printf("%d\n", sin_pitch);
    } else if (LOGMODE == LOG_FREQ) {
        unsigned long us = system_get_time();
        if (++call_counter == FREQUENCY_SAMPLES) {
            os_printf("%lu\n",
                FREQUENCY_SAMPLES * 1000000 / (us - last_call_time));
            call_counter = 0;
            last_call_time = us;
        }
    }
}

void ICACHE_FLASH_ATTR compute(os_event_t *e) {
    if (!mpu_init_succeeded || ota_started) {
        set_motors(0, 0);
        set_both_eyes(LILA);
        return;
    }

    static unsigned long last_battery_check = 0;
    static unsigned int battery_value = 1024;
    static bool send_battery = false;

    unsigned long current_time;

    while (!mpu_read_int_status(MPU_ADDR)) {
        current_time = system_get_time() / 1024;
        if (ENABLE_BATTERY_CHECK &&
            current_time - last_battery_check > BATTERY_CHECK_INTERVAL) {
            last_battery_check = current_time;
            battery_value = q16_exponential_smooth(battery_value, system_adc_read(),
                FLT_TO_Q16(0.25f));
            send_battery = true;

            if (battery_value <
                (unsigned int)(BATTERY_THRESHOLD * BATTERY_CALIBRATION_FACTOR)) {
                set_both_eyes(BLACK);
                // Put MPU6050 to sleep
                mpu_go_to_sleep();
                set_motors(0, 0);
                system_deep_sleep(UINT32_MAX);
                return;
            }
        }
    }

    // Perform MPU quaternion update
    int16_t raw_data[6];
    mpu_read_raw_data(MPU_ADDR, raw_data);
    // Update orientation estimate
    static quaternion_fix quat = { Q16_ONE, 0, 0, 0 };
    madgwick_ahrs_update_imu(&imuparams, &raw_data[0], &raw_data[3],
        &quat);
    // Calculate sine of pitch angle from quaternion
    q16 sin_pitch = -gravity_z(&quat);

    static q16 travel_speed = 0;
    static q16 smoothed_target_speed = 0;

    // Exponential smoothing of target speed
    smoothed_target_speed = q16_exponential_smooth(smoothed_target_speed,
        target_speed, FLT_TO_Q16(TARGET_SPEED_SMOOTHING));

    static state my_state = STABILIZING_ORIENTATION;
    static unsigned long stage_started = 0;
    current_time = system_get_time() / 1024;
    if (my_state == STABILIZING_ORIENTATION) {
        if (current_time - stage_started > ORIENTATION_STABILIZE_DURATION) {
            my_state = RUNNING;
            stage_started = current_time;
        }
    } else if (my_state == RUNNING) {
        if (sin_pitch < FALL_UPPER_BOUND && sin_pitch > FALL_LOWER_BOUND) {
            // Perform PID update
            q16 target_angle = pid_compute(travel_speed, smoothed_target_speed,
                &vel_pid_settings, &vel_pid_state);
            bool use_high_pid =
                sin_pitch < (target_angle - FLT_TO_Q16(HIGH_PID_LIMIT)) ||
                sin_pitch > (target_angle + FLT_TO_Q16(HIGH_PID_LIMIT));
            q16 motor_speed = pid_compute(sin_pitch, target_angle,
                use_high_pid ? &angle_high_pid_settings : &angle_pid_settings,
                &angle_pid_state);

            set_motors(motor_speed + steering_bias, motor_speed - steering_bias);

            // Estimate travel speed by exponential smoothing
            travel_speed = q16_exponential_smooth(travel_speed, motor_speed,
                FLT_TO_Q16(TRAVEL_SPEED_SMOOTHING));
        } else {
            my_state = FALLEN;
            set_both_eyes(BLUE);
            travel_speed = 0;
            set_motors(0, 0);
        }
    } else if (my_state == FALLEN) {
        if (sin_pitch < RECOVER_UPPER_BOUND && sin_pitch > RECOVER_LOWER_BOUND) {
            my_state = RUNNING;
            set_both_eyes(GREEN);
            pid_reset(sin_pitch, 0, &angle_pid_settings, &angle_pid_state);
            pid_reset(0, FLT_TO_Q16(STABLE_ANGLE), &vel_pid_settings,
                &vel_pid_state);
        }
    }

    if (LOGMODE != LOG_NONE) {
        do_log(&raw_data[0], &raw_data[3], sin_pitch);
    }

    if (send_battery) {
        send_battery = false;
        send_battery_reading(battery_value);
    }

    static unsigned long last_sent_quat = 0;
    if (send_quat && current_time - last_sent_quat > QUAT_DELAY) {
        send_quaternion(&quat);
        last_sent_quat = current_time;
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
    {"/flash/upload", upload_firmware, &uploadParams},
    {"/flash/reboot", cgiRebootFirmware, NULL},
#endif

    {"/ws", cgiWebsocket, websocket_connect_cb},
    {"/", cgiRedirect, "/index.html"},
    {"*", cgiEspFsHook, NULL}, //Catch-all cgi function for the filesystem
    {NULL, NULL, NULL}
};

void ICACHE_FLASH_ATTR user_init(void) {
    system_update_cpu_freq(80);

    // Parameter calculation & initialization
    pid_initialize_flt(ANGLE_KP, ANGLE_KI, ANGLE_KD, SAMPLE_TIME,
        -Q16_ONE, Q16_ONE, false, &angle_pid_settings);
    pid_initialize_flt(ANGLE_HIGH_KP, ANGLE_HIGH_KI, ANGLE_HIGH_KD, SAMPLE_TIME,
        -Q16_ONE, Q16_ONE, false, &angle_high_pid_settings);
    pid_reset(0, 0, &angle_pid_settings, &angle_pid_state);
    pid_initialize_flt(VEL_KP, VEL_KI, VEL_KD, SAMPLE_TIME,
        FALL_LOWER_BOUND, FALL_UPPER_BOUND, true, &vel_pid_settings);
    pid_reset(0, 0, &vel_pid_settings, &vel_pid_state);
    calculate_madgwick_params(&imuparams, MADGWICK_BETA,
        2.0f * M_PI / 180.0f * 2000.0f, SAMPLE_TIME);

    uart_init(BIT_RATE_115200, BIT_RATE_115200);
    i2c_gpio_init();
    mpu_init_succeeded = mpu_init();

    wifi_init();

    motors_init();

    eyes_init();
    set_both_eyes(mpu_init_succeeded ? YELLOW : RED);

    // 0x40200000 is the base address for spi flash memory mapping, ESPFS_POS is the position
    // where image is written in flash that is defined in Makefile.
#ifdef ESPFS_POS
    espFsInit((void*)(0x40200000 + ESPFS_POS));
#else
    espFsInit((void*)(webpages_espfs_start));
#endif
    httpdInit(builtInUrls, 80);

    system_os_task(compute, 2, task_queue, QUEUE_LEN);
    system_os_post(2, 0, 0);
}

