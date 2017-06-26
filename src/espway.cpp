extern "C" {
#include <string.h>
#include <espressif/esp_common.h>
#include <dhcpserver.h>
#include <FreeRTOS.h>
#include <task.h>
#include <httpd/httpd.h>
#include <esp8266.h>
#include <esp/uart.h>
#include <sysparam.h>

#include "i2c/i2c.h"
#include "lib/mpu6050.h"
#include "lib/imu.h"
#include "lib/pid.h"
#include "lib/eyes.h"
#include "lib/motors.h"
}

#include "config.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define AP_SSID "ESPway"

#define LOGMODE LOG_NONE

#define FALL_LOWER_BOUND FLT_TO_Q16(STABLE_ANGLE - FALL_LIMIT)
#define FALL_UPPER_BOUND FLT_TO_Q16(STABLE_ANGLE + FALL_LIMIT)
#define RECOVER_LOWER_BOUND FLT_TO_Q16(STABLE_ANGLE - RECOVER_LIMIT)
#define RECOVER_UPPER_BOUND FLT_TO_Q16(STABLE_ANGLE + RECOVER_LIMIT)
#define ROLL_LOWER_BOUND FLT_TO_Q16(-ROLL_LIMIT)
#define ROLL_UPPER_BOUND FLT_TO_Q16(ROLL_LIMIT)

#define BATTERY_COEFFICIENT FLT_TO_Q16(100.0f / BATTERY_CALIBRATION_FACTOR)

#define FREQUENCY_SAMPLES 1024
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
enum state { STABILIZING_ORIENTATION, RUNNING, FALLEN, WOUND_UP };

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

bool mpu_online = false;
bool send_quat = false;
bool ota_started = false;
bool save_config = false;
bool clear_config = false;
bool load_default_config = false;

espway_config my_config;

/* IPAddress apIP(192, 168, 4, 1); */

/* AsyncWebServer server(80); */
/* AsyncWebSocket ws("/ws"); */

/* const uint8_t DNS_PORT = 53; */
/* DNSServer dnsServer; */

void pretty_print_config() {
    printf(
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
        "\n\n",
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
    load_hardcoded_config();
    sysparam_get_data_static("ANGLE_PID", (uint8_t *)&my_config.pid_coeffs_arr[ANGLE],
        sizeof(pid_coeffs), NULL, NULL);
    sysparam_get_data_static("ANGLE_HIGH_PID", (uint8_t *)&my_config.pid_coeffs_arr[ANGLE_HIGH],
        sizeof(pid_coeffs), NULL, NULL);
    sysparam_get_data_static("VEL_PID", (uint8_t *)&my_config.pid_coeffs_arr[VEL],
        sizeof(pid_coeffs), NULL, NULL);
    sysparam_get_data_static("GYRO_OFFSETS", (uint8_t *)&my_config.gyro_offsets,
        3 * sizeof(int16_t), NULL, NULL);
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
    bool success = true;

    success = sysparam_set_data("ANGLE_PID", (uint8_t *)&my_config.pid_coeffs_arr[ANGLE],
        sizeof(pid_coeffs), true);
    if (success) success = sysparam_set_data("ANGLE_HIGH_PID", (uint8_t *)&my_config.pid_coeffs_arr[ANGLE_HIGH], sizeof(pid_coeffs), true);
    if (success) success = sysparam_set_data("VEL_PID", (uint8_t *)&my_config.pid_coeffs_arr[VEL], sizeof(pid_coeffs), true);
    if (success) success = sysparam_set_data("GYRO_OFFSETS", (uint8_t *)&my_config.gyro_offsets, 3 * sizeof(int16_t), true);

    if (success) {
        response = RES_SAVE_CONFIG_SUCCESS;
    } else {
        response = RES_SAVE_CONFIG_FAILURE;
    }
    /* ws.binaryAll(&response, 1); */
    return success;
}

bool clear_flash_config() {
    uint32_t base_addr, num_sectors;
    return sysparam_get_info(&base_addr, &num_sectors) == SYSPARAM_OK &&
        sysparam_create_area(base_addr, num_sectors, true) == SYSPARAM_OK;
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
    /* ws.binaryAll(&response, 1); */
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
    /* ws.binaryAll(payload, 14); */
}

/*
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
*/

void send_quaternion(const quaternion_fix * const quat) {
    uint8_t buf[9];
    buf[0] = RES_QUATERNION;
    int16_t *qdata = (int16_t *)&buf[1];
    qdata[0] = quat->q0 / 2;
    qdata[1] = quat->q1 / 2;
    qdata[2] = quat->q2 / 2;
    qdata[3] = quat->q3 / 2;
    /* ws.binaryAll(buf, 9); */
    send_quat = false;
}

void send_battery_reading(uint16_t battery_reading) {
    uint8_t buf[3];
    buf[0] = BATTERY;
    uint16_t *payload = (uint16_t *)&buf[1];
    payload[0] = q16_mul(battery_reading, BATTERY_COEFFICIENT);
    /* ws.binaryAll(buf, 3); */
}

/*
void loop() {
    if (ota_started) {
        ArduinoOTA.handle();
        return;
    }

    if (!mpu_online) {
        set_motors(0, 0);
        set_both_eyes(RED);
        return;
    }

    static unsigned long last_battery_check = 0;
    static unsigned long mpu_last_online = 0;
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

    while ((mpu_read_int_status(MPU_ADDR) & MPU_DATA_RDY_INT) == 0) {
        if (millis() - mpu_last_online > 100) {
            mpu_online = false;
            return;
        }
        yield();
    }

    // Perform MPU quaternion update
    static int16_t raw_data[6];
    mpu_read_raw_data(MPU_ADDR, raw_data);
    // Update orientation estimate
    static quaternion_fix quat = { Q16_ONE, 0, 0, 0 };
    madgwick_ahrs_update_imu(&imuparams, &raw_data[0], &raw_data[3], &quat);
    // Calculate sine of pitch angle from quaternion
    q16 sin_pitch = -gravity_z(&quat);
    q16 sin_roll = gravity_y(&quat);

    static q16 travel_speed = 0;
    static q16 smoothed_target_speed = 0;

    // Exponential smoothing of target speed
    smoothed_target_speed = q16_exponential_smooth(smoothed_target_speed,
        target_speed, FLT_TO_Q16(TARGET_SPEED_SMOOTHING));

    static state my_state = STABILIZING_ORIENTATION;
    static unsigned long stage_started = 0;
    static unsigned long last_wind_up = 0;
    current_time = millis();
    mpu_last_online = current_time;
    if (my_state == STABILIZING_ORIENTATION) {
        if (current_time - stage_started > ORIENTATION_STABILIZE_DURATION) {
            my_state = RUNNING;
            stage_started = current_time;
        }
    } else if (my_state == RUNNING || my_state == WOUND_UP) {
        if (sin_pitch < FALL_UPPER_BOUND && sin_pitch > FALL_LOWER_BOUND &&
            sin_roll < ROLL_UPPER_BOUND && sin_roll > ROLL_LOWER_BOUND) {
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

            if (motor_speed < FLT_TO_Q16(MOTOR_DEADBAND) &&
                motor_speed > -FLT_TO_Q16(MOTOR_DEADBAND)) {
                motor_speed = 0;
            }

            if (my_state == WOUND_UP) {
                set_motors(0, 0);
            } else {
                set_motors(motor_speed + steering_bias,
                    motor_speed - steering_bias);
            }

            if (motor_speed != Q16_ONE && motor_speed != -Q16_ONE) {
                last_wind_up = current_time;
            } else if (current_time - last_wind_up > WINDUP_TIMEOUT) {
                my_state = WOUND_UP;
                set_both_eyes(BLUE);
            }

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
        if (sin_pitch < RECOVER_UPPER_BOUND &&
            sin_pitch > RECOVER_LOWER_BOUND &&
            sin_roll < ROLL_UPPER_BOUND && sin_roll > ROLL_LOWER_BOUND) {
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

    dnsServer.processNextRequest();
    ArduinoOTA.handle();
}
*/

void wifi_setup(void) {
    sdk_wifi_set_opmode(SOFTAP_MODE);
    struct ip_info ap_ip;
    IP4_ADDR(&ap_ip.ip, 192, 168, 4, 1);
    IP4_ADDR(&ap_ip.gw, 0, 0, 0, 0);
    IP4_ADDR(&ap_ip.netmask, 255, 255, 255, 0);
    sdk_wifi_set_ip_info(1, &ap_ip);

    struct sdk_softap_config ap_config = {};
    strcpy((char *)ap_config.ssid, AP_SSID);
    ap_config.channel = 3;
    ap_config.ssid_len = strlen(AP_SSID);
    ap_config.authmode = AUTH_OPEN;
    ap_config.max_connection = 1;
    ap_config.beacon_interval = 100;
    sdk_wifi_softap_set_config(&ap_config);

    ip_addr_t first_client_ip;
    IP4_ADDR(&first_client_ip, 192, 168, 4, 2);
    dhcpserver_start(&first_client_ip, 4);
}

void httpd_task(void *pvParameters)
{
    /* websocket_register_callbacks((tWsOpenHandler) websocket_open_cb, */
    /*         (tWsHandler) websocket_cb); */
    httpd_init();

    for (;;);
}

void do_loop(void *pvParameters) {
    int16_t raw_data[6];
    uint32_t time_old = 0;
    uint32_t current_time = 0;
    int n = 0;
    quaternion_fix quat = { Q16_ONE, 0, 0, 0 };
    q16 travel_speed = 0;
    q16 smoothed_target_speed = 0;
    state my_state = STABILIZING_ORIENTATION;
    unsigned long stage_started = 0;
    unsigned long last_wind_up = 0;

    for (;;) {
        xTaskNotifyWait(0, 0, NULL, 1);
        mpu_read_raw_data(MPU_ADDR, raw_data);

        // Update orientation estimate
        madgwick_ahrs_update_imu(&imuparams, &raw_data[0], &raw_data[3], &quat);
        // Calculate sine of pitch angle from quaternion
        q16 sin_pitch = -gravity_z(&quat);
        q16 sin_roll = gravity_y(&quat);

        // Exponential smoothing of target speed
        smoothed_target_speed = q16_exponential_smooth(smoothed_target_speed,
            target_speed, FLT_TO_Q16(TARGET_SPEED_SMOOTHING));

        current_time = sdk_system_get_time() / 1024;

        if (my_state == STABILIZING_ORIENTATION) {
            if (current_time - stage_started > ORIENTATION_STABILIZE_DURATION) {
                my_state = RUNNING;
                stage_started = current_time;
            }
        } else if (my_state == RUNNING || my_state == WOUND_UP) {
            if (sin_pitch < FALL_UPPER_BOUND && sin_pitch > FALL_LOWER_BOUND &&
                sin_roll < ROLL_UPPER_BOUND && sin_roll > ROLL_LOWER_BOUND) {
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

                if (motor_speed < FLT_TO_Q16(MOTOR_DEADBAND) &&
                    motor_speed > -FLT_TO_Q16(MOTOR_DEADBAND)) {
                    motor_speed = 0;
                }

                if (my_state == WOUND_UP) {
                    set_motors(0, 0);
                } else {
                    set_motors(motor_speed + steering_bias,
                        motor_speed - steering_bias);
                }

                if (motor_speed != Q16_ONE && motor_speed != -Q16_ONE) {
                    last_wind_up = current_time;
                } else if (current_time - last_wind_up > WINDUP_TIMEOUT) {
                    my_state = WOUND_UP;
                    set_both_eyes(BLUE);
                }

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
            if (sin_pitch < RECOVER_UPPER_BOUND &&
                sin_pitch > RECOVER_LOWER_BOUND &&
                sin_roll < ROLL_UPPER_BOUND && sin_roll > ROLL_LOWER_BOUND) {
                my_state = RUNNING;
                set_both_eyes(GREEN);
                pid_reset(sin_pitch, 0, &pid_settings_arr[ANGLE], &angle_pid_state);
                pid_reset(0, FLT_TO_Q16(STABLE_ANGLE), &pid_settings_arr[VEL],
                    &vel_pid_state);
            }
        }

        if (LOGMODE == LOG_FREQ) {
            n += 1;
            if (n == 1024) {
                n = 0;
                uint32_t looptime = (current_time - time_old) / 1024;
                printf("Looptime: %u\n", looptime);
                time_old = current_time;
            }
        } else if (LOGMODE == LOG_RAW) {
            printf("%d, %d, %d, %d, %d, %d\n",
                raw_data[0], raw_data[1], raw_data[2],
                raw_data[3], raw_data[4], raw_data[5]);
        } else if (LOGMODE == LOG_PITCH) {
        }
    }
}

TaskHandle_t xCalculationTask;

void mpu_interrupt_handler(uint8_t gpio_num) {
    BaseType_t xHigherPriorityTaskHasWoken = pdFALSE;
    xTaskNotifyFromISR(xCalculationTask, 0, eNoAction, &xHigherPriorityTaskHasWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskHasWoken);
}

extern "C" void user_init(void)
{
    uart_set_baud(0, 115200);
    i2c_init(5, 0);
    mpu_online = mpu_init();
    eyes_init();

    motors_init();

    load_stored_config();
    apply_config_params();
    pretty_print_config();

    // Parameter calculation & initialization
    pid_reset(0, 0, &pid_settings_arr[ANGLE], &angle_pid_state);
    pid_reset(0, 0, &pid_settings_arr[VEL], &vel_pid_state);
    calculate_madgwick_params(&imuparams, MADGWICK_BETA,
        2.0f * M_PI / 180.0f * 2000.0f, SAMPLE_TIME);

    set_both_eyes(mpu_online ? YELLOW : RED);

    wifi_setup();

    // OTA init

    // DNS server

    xTaskCreate(&httpd_task, "HTTP Daemon", 128, NULL, 2, NULL);
    xTaskCreate(&do_loop, "Main loop", 256, NULL, 3, &xCalculationTask);

    gpio_enable(4, GPIO_INPUT);
    gpio_set_interrupt(4, GPIO_INTTYPE_EDGE_POS, mpu_interrupt_handler);
}

