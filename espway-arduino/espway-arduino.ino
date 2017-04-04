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

#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <brzo_i2c.h>
#include <ArduinoOTA.h>

#include "flash_config.h"
#include "motors.h"
#include "eyes.h"

extern "C" {
#include "mpu6050.h"
#include "imu.h"
#include "pid.h"
}

#include "config.h"

#define LOGMODE LOG_NONE

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

enum logmode { LOG_FREQ, LOG_RAW, LOG_PITCH, LOG_NONE };
enum state { STABILIZING_ORIENTATION, RUNNING, FALLEN };

enum ws_msg_t {
    STEERING = 0,

    REQ_QUATERNION = 1,
    RES_QUATERNION = 2,

    BATTERY = 3,
    BATTERY_CUTOFF = 4,

    REQ_SET_PID_PARAMS = 5,
    RES_SET_PID_PARAMS_ACK = 6,
    REQ_GET_PID_PARAMS = 7,
    RES_PID_PARAMS = 8,

    REQ_SET_GYRO_OFFSETS = 9,
    RES_SET_GYRO_OFFSETS_FAILURE = 10,
    RES_SET_GYRO_OFFSETS_SUCCESS = 11,

    REQ_SAVE_CONFIG = 12,
    RES_SAVE_CONFIG_FAILURE = 13,
    RES_SAVE_CONFIG_SUCCESS = 14,

    REQ_CLEAR_CONFIG = 15,
    RES_CLEAR_CONFIG_FAILURE = 16,
    RES_CLEAR_CONFIG_SUCCESS = 17,

    REQ_LOAD_FLASH_CONFIG = 18,
    RES_LOAD_FLASH_CONFIG_DONE = 19,

    REQ_ENABLE_MOTORS = 20,
    REQ_DISABLE_MOTORS = 21
};

enum pid_controller_index {
    ANGLE = 0,
    ANGLE_HIGH = 1,
    VEL = 2
};

struct espway_config {
    pid_coeffs pid_coeffs_arr[3];
    int16_t gyro_offsets[3];
};

const espway_config DEFAULT_CONFIG = {
    .pid_coeffs_arr = {
        { FLT_TO_Q16(ANGLE_KP), FLT_TO_Q16(ANGLE_KI), FLT_TO_Q16(ANGLE_KD) },
        { FLT_TO_Q16(ANGLE_HIGH_KP), FLT_TO_Q16(ANGLE_HIGH_KI), FLT_TO_Q16(ANGLE_HIGH_KD) },
        { FLT_TO_Q16(VEL_KP), FLT_TO_Q16(VEL_KI), FLT_TO_Q16(VEL_KD) }
    },
    .gyro_offsets = { GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET }
};

madgwickparams imuparams;
pidstate vel_pid_state;
pidstate angle_pid_state;

pidsettings pid_settings_arr[3];

q16 target_speed = 0;
q16 steering_bias = 0;

bool mpu_init_succeeded = false;
bool send_quat = false;
bool ota_started = false;
bool save_config = false;
bool clear_config = false;
bool load_default_config = false;

espway_config my_config;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void pretty_print_config() {
    Serial.printf_P(PSTR(
            "\n\nESPway current config:\n\n"
            "#define ANGLE_KP %d\n"
            "#define ANGLE_KI %d\n"
            "#define ANGLE_KD %d\n"
            "#define ANGLE_HIGH_KP %d\n"
            "#define ANGLE_HIGH_KI %d\n"
            "#define ANGLE_HIGH_KD %d\n"
            "#define VEL_KP %d\n"
            "#define VEL_KI %d\n"
            "#define VEL_KD %d\n"
            "#define GYRO_X_OFFSET %d\n"
            "#define GYRO_Y_OFFSET %d\n"
            "#define GYRO_Z_OFFSET %d\n"
            "\n\n"
        ),
        my_config.pid_coeffs_arr[ANGLE].p,
        my_config.pid_coeffs_arr[ANGLE].i,
        my_config.pid_coeffs_arr[ANGLE].d,
        my_config.pid_coeffs_arr[ANGLE_HIGH].p,
        my_config.pid_coeffs_arr[ANGLE_HIGH].i,
        my_config.pid_coeffs_arr[ANGLE_HIGH].d,
        my_config.pid_coeffs_arr[VEL].p,
        my_config.pid_coeffs_arr[VEL].i,
        my_config.pid_coeffs_arr[VEL].d,
        my_config.gyro_offsets[0],
        my_config.gyro_offsets[1],
        my_config.gyro_offsets[2]
    );
}

void load_hardcoded_config() {
    my_config = DEFAULT_CONFIG;
}

void load_stored_config() {
    if (!read_flash_config<espway_config>(my_config, CONFIG_VERSION)) {
        load_hardcoded_config();
    }
}

void apply_config_params() {
    pid_initialize(&my_config.pid_coeffs_arr[ANGLE],
        FLT_TO_Q16(SAMPLE_TIME),
        -Q16_ONE, Q16_ONE, false, &pid_settings_arr[ANGLE]);
    pid_initialize(&my_config.pid_coeffs_arr[ANGLE_HIGH],
        FLT_TO_Q16(SAMPLE_TIME),
        -Q16_ONE, Q16_ONE, false, &pid_settings_arr[ANGLE_HIGH]);
    pid_initialize(&my_config.pid_coeffs_arr[VEL],
        FLT_TO_Q16(SAMPLE_TIME), FALL_LOWER_BOUND, FALL_UPPER_BOUND, true,
        &pid_settings_arr[VEL]);

    mpu_set_gyro_offsets(my_config.gyro_offsets);
}

bool do_save_config() {
    uint8_t response;
    bool success = write_flash_config<espway_config>(my_config, CONFIG_VERSION);
    if (success) {
        response = RES_SAVE_CONFIG_SUCCESS;
    } else {
        response = RES_SAVE_CONFIG_FAILURE;
    }
    ws.binaryAll(&response, 1);
    return success;
}

bool do_clear_config() {
    uint8_t response;
    // Clear the configuration by writing config version zero
    bool success = clear_flash_config();
    if (success) {
        response = RES_CLEAR_CONFIG_SUCCESS;
        load_hardcoded_config();
    } else {
        response = RES_CLEAR_CONFIG_FAILURE;
    }
    ws.binaryAll(&response, 1);
    return success;
}

void update_pid_controller(pid_controller_index idx, q16 p, q16 i, q16 d) {
    if (idx > 2) {
        return;
    }

    pid_coeffs *p_coeffs = &my_config.pid_coeffs_arr[idx];
    p_coeffs->p = p;
    p_coeffs->i = i;
    p_coeffs->d = d;
    pid_update_params(p_coeffs, &pid_settings_arr[idx]);

    if (idx == ANGLE) {
        // If ANGLE PID coefficients are updated, automatically update the
        // high gain PID
        pid_coeffs *p_coeffs = &my_config.pid_coeffs_arr[ANGLE_HIGH];
        p_coeffs->p = q16_mul(FLT_TO_Q16(1.5f), p);
        p_coeffs->i = i;
        p_coeffs->d = d;
        pid_update_params(p_coeffs, &pid_settings_arr[ANGLE_HIGH]);
    }
}

void send_pid_params(pid_controller_index idx) {
    uint8_t payload[14];
    payload[0] = RES_PID_PARAMS;
    payload[1] = idx;
    int32_t *params = (int32_t *)(&payload[2]);
    params[0] = my_config.pid_coeffs_arr[idx].p;
    params[1] = my_config.pid_coeffs_arr[idx].i;
    params[2] = my_config.pid_coeffs_arr[idx].d;
    ws.binaryAll(payload, 14);
}

void websocket_recv_cb(AsyncWebSocket * server, AsyncWebSocketClient * client,
    AwsEventType type, void * arg, uint8_t *data, size_t len) {
    if (len == 0 || type != WS_EVT_DATA) {
        return;
    }
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    if (!info->final || info->index != 0 || info->len != len ||
        info->opcode != WS_BINARY) {
        return;
    }

    uint8_t msgtype = data[0];
    uint8_t *payload = &data[1];
    int data_len = len - 1;
    int8_t *signed_data;
    pid_controller_index pid_index;
    int32_t *i32_data;
    uint8_t res;

    switch (msgtype) {
        case STEERING:
            // Parameters: velocity (int8_t), turn rate (int8_t)
            if (data_len != 2) {
                break;
            }
            signed_data = (int8_t *)payload;
            steering_bias = (FLT_TO_Q16(STEERING_FACTOR) * signed_data[0]) / 128;
            target_speed = (FLT_TO_Q16(SPEED_CONTROL_FACTOR) * signed_data[1]) / 128;
            break;

        case REQ_QUATERNION:
            send_quat = true;
            break;

        case REQ_SET_PID_PARAMS:
            // Parameters: pid index (uint8_t), P (q16/int32_t), I (q16), D (q16)
            if (data_len != 13 || load_default_config) {
                // Ignore the request if we are about to load the default config
                break;
            }

            pid_index = (pid_controller_index)payload[0];
            i32_data = (int32_t *)(&payload[1]);
            if (pid_index <= 2) {
                update_pid_controller(pid_index, i32_data[0], i32_data[1],
                    i32_data[2]);

                res = RES_SET_PID_PARAMS_ACK;
                ws.binaryAll(&res, 1);
            }

            break;

        case REQ_GET_PID_PARAMS:
            if (data_len != 1) {
                break;
            }

            if (payload[0] <= 2) {
                send_pid_params((pid_controller_index)payload[0]);
            }

            break;

        case REQ_LOAD_FLASH_CONFIG:
            if (data_len != 0) {
                break;
            }

            load_default_config = true;

            break;

        case REQ_SAVE_CONFIG:
            if (data_len != 0) {
                break;
            }
            save_config = true;
            break;

        case REQ_CLEAR_CONFIG:
            if (data_len != 0) {
                break;
            }
            clear_config = true;
            break;
    }
}

void send_quaternion(const quaternion_fix * const quat) {
    uint8_t buf[9];
    buf[0] = RES_QUATERNION;
    int16_t *qdata = (int16_t *)&buf[1];
    qdata[0] = quat->q0 / 2;
    qdata[1] = quat->q1 / 2;
    qdata[2] = quat->q2 / 2;
    qdata[3] = quat->q3 / 2;
    ws.binaryAll(buf, 9);
    send_quat = false;
}

void send_battery_reading(uint16_t battery_reading) {
    uint8_t buf[3];
    buf[0] = BATTERY;
    uint16_t *payload = (uint16_t *)&buf[1];
    payload[0] = q16_mul(battery_reading, BATTERY_COEFFICIENT);
    ws.binaryAll(buf, 3);
}

void do_log(int16_t *raw_accel, int16_t *raw_gyro, q16 sin_pitch) {
    static unsigned long last_call_time = 0;
    static unsigned int call_counter = 0;

    if (LOGMODE == LOG_RAW) {
        Serial.printf("%d, %d, %d, %d, %d, %d\n",
            raw_accel[0], raw_accel[1], raw_accel[2],
            raw_gyro[0], raw_gyro[1], raw_gyro[2]);
    } else if (LOGMODE == LOG_PITCH) {
        Serial.printf("%d\n", sin_pitch);
    } else if (LOGMODE == LOG_FREQ) {
        unsigned long ms = millis();
        if (++call_counter == FREQUENCY_SAMPLES) {
            Serial.printf("%lu\n",
                FREQUENCY_SAMPLES * 1000 / (ms - last_call_time));
            call_counter = 0;
            last_call_time = ms;
        }
    }
}

void loop() {
    if (ota_started) {
        ArduinoOTA.handle();
        return;
    }

    if (!mpu_init_succeeded) {
        set_motors(0, 0);
        return;
    }

    static unsigned long last_battery_check = 0;
    static unsigned int battery_value = 1024;
    static bool send_battery = false;

    unsigned long current_time = millis();

    if (ENABLE_BATTERY_CHECK &&
        current_time - last_battery_check > BATTERY_CHECK_INTERVAL) {
        last_battery_check = current_time;
        battery_value = q16_exponential_smooth(battery_value, analogRead(A0),
            FLT_TO_Q16(0.25f));
        send_battery = true;

        if (ENABLE_BATTERY_CUTOFF && battery_value <
            (unsigned int)(BATTERY_THRESHOLD * BATTERY_CALIBRATION_FACTOR)) {
            set_both_eyes(BLACK);
            // Put MPU6050 to sleep
            mpu_go_to_sleep();
            set_motors(0, 0);
            ESP.deepSleep(UINT32_MAX);
            return;
        }
    }

    while (!mpu_read_int_status(MPU_ADDR)) { yield(); }

    // Perform MPU quaternion update
    static int16_t raw_data[6];
    mpu_read_raw_data(MPU_ADDR, raw_data);
    // Update orientation estimate
    static quaternion_fix quat = { Q16_ONE, 0, 0, 0 };
    madgwick_ahrs_update_imu(&imuparams, &raw_data[0], &raw_data[3], &quat);
    // Calculate sine of pitch angle from quaternion
    q16 sin_pitch = -gravity_z(&quat);

    static q16 travel_speed = 0;
    static q16 smoothed_target_speed = 0;

    // Exponential smoothing of target speed
    smoothed_target_speed = q16_exponential_smooth(smoothed_target_speed,
        target_speed, FLT_TO_Q16(TARGET_SPEED_SMOOTHING));

    static state my_state = STABILIZING_ORIENTATION;
    static unsigned long stage_started = 0;
    current_time = millis();
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

    if (save_config) {
        do_save_config();
        save_config = false;
    }

    if (clear_config) {
        do_clear_config();
        clear_config = false;
    }

    if (load_default_config) {
        load_stored_config();
        uint8_t response = RES_LOAD_FLASH_CONFIG_DONE;
        ws.binaryAll(&response, 1);
        load_default_config = false;
    }

    ArduinoOTA.handle();
}

void wifi_init() {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(
        IPAddress(192, 168, 4, 1),   // local IP
        IPAddress(192, 168, 4, 1),   // gateway
        IPAddress(255, 255, 255, 0)  // mask
    );
    WiFi.softAP(WIFI_SSID, "", WIFI_CHANNEL, 0, 1);
}

void onRequest(AsyncWebServerRequest *request) {
    //Handle Unknown Request
    request->send(404);
}

void onBody(AsyncWebServerRequest *request, uint8_t *, size_t, size_t, size_t) {
    //Handle body
    request->send(404);
}

void onUpload(AsyncWebServerRequest *request, String, size_t, uint8_t *,
    size_t, bool) {
    //Handle upload
    request->send(404);
}

void setup() {
    motors_init();

    Serial.begin(115200);

    flash_config_begin();

    brzo_i2c_setup(4, 5, 2000);
    mpu_init_succeeded = mpu_init();

    load_stored_config();
    apply_config_params();
    pretty_print_config();

    // Parameter calculation & initialization
    pid_reset(0, 0, &pid_settings_arr[ANGLE], &angle_pid_state);
    pid_reset(0, 0, &pid_settings_arr[VEL], &vel_pid_state);
    calculate_madgwick_params(&imuparams, MADGWICK_BETA,
        2.0f * M_PI / 180.0f * 2000.0f, SAMPLE_TIME);

    pinMode(A0, INPUT);
    eyes_init();
    set_both_eyes(mpu_init_succeeded ? YELLOW : RED);

    wifi_init();

    SPIFFS.begin();
    ws.onEvent(websocket_recv_cb);
    server.addHandler(&ws);
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
    server.onNotFound(onRequest);
    server.onFileUpload(onUpload);
    server.onRequestBody(onBody);
    server.begin();

    ArduinoOTA.onStart([]() {
        ota_started = true;
        set_motors(0, 0);
        set_both_eyes(LILA);
    });
    ArduinoOTA.begin();
}

