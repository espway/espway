/*
    Firmware for a segway-style robot using ESP8266.
    Copyright (C) 2017  Sakari Kapanen

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

// libesphttpd includes
#include "httpd.h"
#include "cgiflash.h"
#include "httpdespfs.h"
#include "espfs.h"
#include "webpages-espfs.h"
#include "cgiwebsocket.h"

#include "brzo_i2c.h"

// internal includes
#include "mpu6050.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"
#include "eyes.h"
#include "flash_config.h"

#include "config.h"

#define QUEUE_LEN 1
#define M_PI 3.14159265359f

#define LOGMODE LOG_FREQ

#define FALL_LOWER_BOUND FLT_TO_Q16(STABLE_ANGLE - FALL_LIMIT)
#define FALL_UPPER_BOUND FLT_TO_Q16(STABLE_ANGLE + FALL_LIMIT)
#define RECOVER_LOWER_BOUND FLT_TO_Q16(STABLE_ANGLE - RECOVER_LIMIT)
#define RECOVER_UPPER_BOUND FLT_TO_Q16(STABLE_ANGLE + RECOVER_LIMIT)

#define BATTERY_COEFFICIENT FLT_TO_Q16(100.0f / BATTERY_CALIBRATION_FACTOR)

#define FREQUENCY_SAMPLES 1000
#define QUAT_DELAY 50

#define SAMPLE_TIME ((1.0f + MPU_RATE) / 1000.0f)

#define CONFIG_VERSION 2

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

    BATTERY = 3,
    BATTERY_CUTOFF = 4,

    REQ_SET_PID_PARAMS = 5,
    REQ_GET_PID_PARAMS = 6,
    RES_PID_PARAMS = 7,

    REQ_SET_GYRO_OFFSETS = 8,
    RES_SET_GYRO_OFFSETS_FAILURE = 9,
    RES_SET_GYRO_OFFSETS_SUCCESS = 10,

    REQ_SAVE_CONFIG = 11,
    RES_SAVE_CONFIG_FAILURE = 12,
    RES_SAVE_CONFIG_SUCCESS = 13
} ws_msg_type;

typedef enum {
    ANGLE = 0,
    ANGLE_HIGH = 1,
    VEL = 2
} pid_controller_index;

typedef struct {
    q16 p, i, d;
} pid_coeffs;

typedef struct {
    pid_coeffs pid_coeffs_arr[3];
    int16_t gyro_offsets[3];
} espway_config;

madgwickparams imuparams;
pidstate vel_pid_state;
pidstate angle_pid_state;

pidsettings pid_settings_arr[3];

q16 target_speed = 0;
q16 steering_bias = 0;

bool mpu_init_succeeded = false;
bool send_quat = false;
bool ota_started = false;

espway_config my_config;

void apply_config_params() {
    pid_initialize(
        my_config.pid_coeffs_arr[ANGLE].p,
        my_config.pid_coeffs_arr[ANGLE].i,
        my_config.pid_coeffs_arr[ANGLE].d,
        FLT_TO_Q16(SAMPLE_TIME),
        -Q16_ONE, Q16_ONE, false, &pid_settings_arr[ANGLE]);
    pid_initialize(
        my_config.pid_coeffs_arr[ANGLE_HIGH].p,
        my_config.pid_coeffs_arr[ANGLE_HIGH].i,
        my_config.pid_coeffs_arr[ANGLE_HIGH].d,
        FLT_TO_Q16(SAMPLE_TIME),
        -Q16_ONE, Q16_ONE, false, &pid_settings_arr[ANGLE_HIGH]);
    pid_initialize(
        my_config.pid_coeffs_arr[VEL].p,
        my_config.pid_coeffs_arr[VEL].i,
        my_config.pid_coeffs_arr[VEL].d,
        FLT_TO_Q16(SAMPLE_TIME), FALL_LOWER_BOUND, FALL_UPPER_BOUND, true,
        &pid_settings_arr[VEL]);

    mpu_set_gyro_offsets(my_config.gyro_offsets);
}

void update_pid_controller(pid_controller_index idx, q16 p, q16 i, q16 d) {
    if (idx > 2) {
        return;
    }

    pid_update_params(p, i, d, &pid_settings_arr[idx]);
    pid_coeffs *p_coeffs = &my_config.pid_coeffs_arr[idx];
    p_coeffs->p = p;
    p_coeffs->i = i;
    p_coeffs->d = d;
}

int upload_firmware(HttpdConnData *connData) {
    ota_started = true;
    return cgiUploadFirmware(connData);
}

void send_pid_params(pid_controller_index idx) {
    uint8_t payload[14];
    payload[0] = RES_PID_PARAMS;
    payload[1] = idx;
    int32_t *params = (int32_t *)(&payload[2]);
    params[0] = my_config.pid_coeffs_arr[idx].p;
    params[1] = my_config.pid_coeffs_arr[idx].i;
    params[2] = my_config.pid_coeffs_arr[idx].d;
    printf("sent %d, %d, %d\n", params[0], params[1], params[2]);
    cgiWebsockBroadcast("/ws", (char *)payload, 14, WEBSOCK_FLAG_BIN);
}

void websocket_recv_cb(Websock *ws, char *signed_data, int len, int flags) {
    if (len == 0) {
        return;
    }
    uint8_t *data = (uint8_t *)signed_data;
    uint8_t msgtype = data[0];
    uint8_t *payload = &data[1];
    int data_len = len - 1;

    switch (msgtype) {
        case STEERING:
            // Parameters: velocity (int8_t), turn rate (int8_t)
            if (data_len != 2) {
                break;
            }
            int8_t *signed_data = (int8_t *)payload;
            steering_bias = (FLT_TO_Q16(STEERING_FACTOR) * signed_data[0]) / 128;
            target_speed = (FLT_TO_Q16(SPEED_CONTROL_FACTOR) * signed_data[1]) / 128;
            break;

        case REQ_QUATERNION:
            send_quat = true;
            break;

        case REQ_SET_PID_PARAMS:
            // Parameters: pid index (uint8_t), P (q16/int32_t), I (q16), D (q16)
            if (data_len != 13) {
                break;
            }

            uint8_t pid_index = payload[0];
            int32_t *i32_data = (int32_t *)(&payload[1]);
            if (pid_index <= 2) {
                update_pid_controller(pid_index, i32_data[0], i32_data[1],
                    i32_data[2]);
                send_pid_params(pid_index);
            }

            break;

        case REQ_GET_PID_PARAMS:
            if (data_len != 1) {
                break;
            }

            if (payload[0] <= 2) {
                send_pid_params(payload[0]);
            }

            break;
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

            if (ENABLE_BATTERY_CUTOFF && battery_value <
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
                &pid_settings_arr[VEL], &vel_pid_state);
            bool use_high_pid =
                sin_pitch < (target_angle - FLT_TO_Q16(HIGH_PID_LIMIT)) ||
                sin_pitch > (target_angle + FLT_TO_Q16(HIGH_PID_LIMIT));
            q16 motor_speed = pid_compute(sin_pitch, target_angle,
                use_high_pid ? &pid_settings_arr[ANGLE_HIGH] :
                               &pid_settings_arr[ANGLE],
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
            pid_reset(sin_pitch, 0, &pid_settings_arr[ANGLE], &angle_pid_state);
            pid_reset(0, FLT_TO_Q16(STABLE_ANGLE), &pid_settings_arr[VEL],
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
    wifi_softap_dhcps_stop();

    struct ip_info info;
    IP4_ADDR(&info.ip, 192, 168, 4, 1);
    IP4_ADDR(&info.gw, 192, 168, 4, 1);
    IP4_ADDR(&info.netmask, 255, 255, 255, 0);
    wifi_set_ip_info(SOFTAP_IF, &info);

    wifi_softap_dhcps_start();

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
    uart_init(BIT_RATE_115200, BIT_RATE_115200);

    brzo_i2c_setup(2000);
    mpu_init_succeeded = mpu_init();

    if (!read_flash_config(&my_config, sizeof(espway_config), CONFIG_VERSION)) {
        // Load default parameters
        my_config.pid_coeffs_arr[ANGLE].p = FLT_TO_Q16(ANGLE_KP);
        my_config.pid_coeffs_arr[ANGLE].i = FLT_TO_Q16(ANGLE_KI);
        my_config.pid_coeffs_arr[ANGLE].d = FLT_TO_Q16(ANGLE_KD);
        my_config.pid_coeffs_arr[ANGLE_HIGH].p = FLT_TO_Q16(ANGLE_HIGH_KP);
        my_config.pid_coeffs_arr[ANGLE_HIGH].i = FLT_TO_Q16(ANGLE_HIGH_KI);
        my_config.pid_coeffs_arr[ANGLE_HIGH].d = FLT_TO_Q16(ANGLE_HIGH_KD);
        my_config.pid_coeffs_arr[VEL].p = FLT_TO_Q16(VEL_KP);
        my_config.pid_coeffs_arr[VEL].i = FLT_TO_Q16(VEL_KI);
        my_config.pid_coeffs_arr[VEL].d = FLT_TO_Q16(VEL_KD);
        os_memcpy(my_config.gyro_offsets, GYRO_OFFSETS, 3 * sizeof(int16_t));
    }
    apply_config_params();

    // Parameter calculation & initialization
    pid_reset(0, 0, &pid_settings_arr[ANGLE], &angle_pid_state);
    pid_reset(0, 0, &pid_settings_arr[VEL], &vel_pid_state);
    calculate_madgwick_params(&imuparams, MADGWICK_BETA,
        2.0f * M_PI / 180.0f * 2000.0f, SAMPLE_TIME);

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

