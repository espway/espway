/*
 * Firmware for a segway-style robot using ESP8266.
 * Copyright (C) 2017  Sakari Kapanen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

extern "C" {
#include <string.h>
#include <espressif/esp_common.h>
#include <user_exception.h>
#include <dhcpserver.h>
#include <esp8266.h>
#include <esp/uart.h>

#include "lib/eyes.h"
#include "lib/delta_sigma.h"
#include "lib/imu_hal.h"
#include "lib/imu_math.h"
#include "lib/motors.h"
}
#include "espway.h"
#include "espway_config.h"
#include "lib/locks.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define AP_SSID "ESPway"

#define LOGMODE LOG_NONE

#define FREQUENCY_SAMPLES 1024
#define QUAT_DELAY 50

#define CONFIG_VERSION 2

#define ORIENTATION_STABILIZE_DURATION_US ((ORIENTATION_STABILIZE_DURATION) * 1000)
#define WINDUP_TIMEOUT_US ((WINDUP_TIMEOUT) * 1000)

#define STEERING_TIMEOUT_MS 1000
#define IMU_TIMEOUT_MS      100

#define PRIO_COMMUNICATION  2
#define PRIO_MAIN_LOOP      (TCPIP_THREAD_PRIO + 1)

typedef enum { LOG_FREQ, LOG_RAW, LOG_PITCH, LOG_NONE } logmode;

static TaskHandle_t xCalculationTask = NULL;
static TaskHandle_t xSteeringWatcher = NULL;
static TaskHandle_t xIMUWatcher = NULL;

static mahony_filter_state imuparams;
static pidstate vel_pid_state;
static pidstate angle_pid_state;

SemaphoreHandle_t pid_mutex;
pidsettings pid_settings_arr[2];

static SemaphoreHandle_t steering_mutex;
static q16 target_speed = 0;
static q16 steering_bias = 0;

static SemaphoreHandle_t orientation_mutex;
static vector3d_fix gravity = {{ 0, 0, Q16_ONE }};
static q16 sin_pitch, sin_roll;

orientation get_orientation()
{
  orientation copy_orientation = {};
  {
    MutexLock lock(orientation_mutex);
    copy_orientation.sin_pitch = sin_pitch;
    copy_orientation.sin_roll = sin_roll;
  }
  return copy_orientation;
}

void set_steering(q16 new_target_speed, q16 new_steering_bias)
{
  {
    MutexLock lock(steering_mutex);
    target_speed = new_target_speed;
    steering_bias = new_steering_bias;
  }

  if (xSteeringWatcher) xTaskNotify(xSteeringWatcher, 0, eNoAction);
}

void battery_cutoff()
{
  set_both_eyes(BLACK);
  set_motors(0, 0);
  sdk_system_deep_sleep(UINT32_MAX);
}

static state my_state;
state get_state()
{
  return my_state;
}

static void main_loop(void *pvParameters)
{
  int16_t raw_data[6];
  uint32_t time_old = 0;
  uint32_t current_time = 0;
  int n = 0;
  q16 travel_speed = 0;
  q16 smoothed_target_speed = 0;
  my_state = STABILIZING_ORIENTATION;
  unsigned long stage_started = 0;
  unsigned long last_wind_up = 0;

  for (;;)
  {
    xTaskNotifyWait(0, 0, NULL, 1);
    int ret = imu_read_raw_data(raw_data);
    if (ret != 0)
      printf("Reading IMU failed with code %d\n", ret);

    // Update orientation estimate
    {
      MutexLock lock(orientation_mutex);
      mahony_filter_update(&imuparams, &raw_data[0], &raw_data[3], &gravity);
      // Calculate sine of pitch angle from gravity vector
      sin_pitch = -gravity.data[IMU_FORWARD_AXIS];
      if (IMU_INVERT_FORWARD_AXIS) sin_pitch = -sin_pitch;
      sin_roll = -gravity.data[IMU_SIDE_AXIS];
      if (IMU_INVERT_SIDE_AXIS) sin_roll = -sin_roll;
    }

    // Exponential smoothing of target speed
    q16 motor_bias;
    {
      MutexLock lock(orientation_mutex);
      smoothed_target_speed = q16_exponential_smooth(smoothed_target_speed,
          target_speed, FLT_TO_Q16(TARGET_SPEED_SMOOTHING));
      motor_bias = steering_bias;
    }

    current_time = sdk_system_get_time();

    if (my_state == STABILIZING_ORIENTATION &&
        current_time - stage_started > ORIENTATION_STABILIZE_DURATION_US)
    {
      my_state = RUNNING;
      stage_started = current_time;
      imuparams.Kp = FLT_TO_Q16(MAHONY_FILTER_KP);
    }
    else if (my_state == RUNNING || my_state == WOUND_UP)
    {
      if (abs(sin_pitch - STABLE_ANGLE) < FALL_LIMIT &&
          abs(sin_roll) < ROLL_LIMIT)
      {
        // Perform PID update
        q16 target_angle, motor_speed;
        {
          MutexLock lock(pid_mutex);
          target_angle = pid_compute(travel_speed, smoothed_target_speed,
              &pid_settings_arr[VEL], &vel_pid_state);

          if (sin_pitch < (target_angle - FLT_TO_Q16(HIGH_PID_LIMIT)))
            motor_speed = -Q16_ONE;
          else if (sin_pitch > target_angle + FLT_TO_Q16(HIGH_PID_LIMIT))
            motor_speed = Q16_ONE;
          else
            motor_speed = pid_compute(sin_pitch, target_angle,
                &pid_settings_arr[ANGLE], &angle_pid_state);
        }

        if (abs(motor_speed) < FLT_TO_Q16(MOTOR_DEADBAND))
          motor_speed = 0;

        if (my_state == WOUND_UP)
          set_motors(0, 0);
        else
          set_motors(motor_speed + motor_bias,
              motor_speed - motor_bias);

        if (motor_speed != Q16_ONE && motor_speed != -Q16_ONE)
          last_wind_up = current_time;
        else if (current_time - last_wind_up > WINDUP_TIMEOUT_US)
        {
          my_state = WOUND_UP;
          set_both_eyes(BLUE);
        }

        // Estimate travel speed by exponential smoothing
        travel_speed = q16_exponential_smooth(travel_speed, motor_speed,
            FLT_TO_Q16(TRAVEL_SPEED_SMOOTHING));
      }
      else
      {
        my_state = FALLEN;
        set_both_eyes(BLUE);
        travel_speed = 0;
        smoothed_target_speed = 0;
        set_motors(0, 0);
      }
    }
    else if (my_state == FALLEN &&
      abs(sin_pitch - STABLE_ANGLE) < RECOVER_LIMIT &&
      abs(sin_roll) < ROLL_LIMIT)
    {
      my_state = RUNNING;
      last_wind_up = current_time;
      set_both_eyes(GREEN);
      pid_reset(sin_pitch, 0, &pid_settings_arr[ANGLE], &angle_pid_state);
      pid_reset(0, FLT_TO_Q16(STABLE_ANGLE), &pid_settings_arr[VEL],
          &vel_pid_state);
    }

    if (LOGMODE == LOG_FREQ)
    {
      n += 1;
      if (n == 1024)
      {
        n = 0;
        uint32_t looptime = (current_time - time_old) / 1024;
        printf("Looptime: %u us\n", looptime);
        time_old = current_time;
      }
    }
    else if (LOGMODE == LOG_RAW)
    {
      printf("%d, %d, %d, %d, %d, %d\n",
          raw_data[0], raw_data[1], raw_data[2],
          raw_data[3], raw_data[4], raw_data[5]);
    }
    else if (LOGMODE == LOG_PITCH)
    {
      printf("%d, %d\n", sin_pitch, sin_roll);
    }

    xTaskNotify(xIMUWatcher, 0, eNoAction);
  }
}

static void imu_interrupt_handler(uint8_t gpio_num)
{
  BaseType_t xHigherPriorityTaskHasWoken = pdFALSE;
  if (xCalculationTask) xTaskNotifyFromISR(xCalculationTask, 0, eNoAction, &xHigherPriorityTaskHasWoken);
  portEND_SWITCHING_ISR(xHigherPriorityTaskHasWoken);
}

static void steering_watcher(void *arg)
{
  for (;;)
    if (!xTaskNotifyWait(0, 0, NULL, STEERING_TIMEOUT_MS / portTICK_PERIOD_MS))
    {
      steering_bias = 0;
      target_speed = 0;
    }
}

static void imu_watcher(void *arg)
{
  for (;;) if (!xTaskNotifyWait(0, 0, NULL, IMU_TIMEOUT_MS / portTICK_PERIOD_MS))
    abort();
}

static void IRAM espway_exception_handler()
{
  _xt_isr_mask(BIT(INUM_TIMER_FRC1));  // Shut down the timer driving the motors
  // Make sure that the motor outputs are enabled as outputs and drive them low
  for (uint8_t i = 12; i <= 15; ++i)
  {
    gpio_enable(i, GPIO_OUTPUT);
    gpio_write(i, 0);
  }
}

static void wifi_setup()
{
  sdk_wifi_set_opmode(SOFTAP_MODE);
  struct ip_info ap_ip;
  IP4_ADDR(&ap_ip.ip, 10, 0, 0, 1);
  IP4_ADDR(&ap_ip.gw, 10, 0, 0, 1);
  IP4_ADDR(&ap_ip.netmask, 255, 255, 255, 0);
  sdk_wifi_set_ip_info(SOFTAP_IF, &ap_ip);

  struct sdk_softap_config ap_config = {};
  strcpy((char *)ap_config.ssid, AP_SSID);
  ap_config.channel = WIFI_CHANNEL;
  ap_config.ssid_len = strlen(AP_SSID);
  ap_config.authmode = AUTH_OPEN;
  ap_config.max_connection = 1;
  ap_config.beacon_interval = 100;
  sdk_wifi_softap_set_config(&ap_config);

  ip_addr_t first_client_ip;
  IP4_ADDR(&first_client_ip, 10, 0, 0, 2);
  dhcpserver_start(&first_client_ip, 1);
}

extern "C" void user_init(void)
{
  set_user_exception_handler(espway_exception_handler);
  sdk_system_update_cpu_freq(SYS_CPU_160MHZ);

  uart_set_baud(0, 115200);
  imu_i2c_configure(I2C_FREQ_1300K, IMU_SCL_PIN, IMU_SDA_PIN);
  int ret = imu_init();
  if (ret != 0)
    printf("IMU init failed with code %d\n", ret);
  eyes_init();
  set_both_eyes(ret == 0 ? YELLOW : RED);

  pid_mutex = xSemaphoreCreateMutex();
  orientation_mutex = xSemaphoreCreateMutex();
  steering_mutex = xSemaphoreCreateMutex();

  motors_init();

  load_config();
  apply_config_params();
  pretty_print_config();

  // Parameter calculation & initialization
  pid_reset(0, 0, &pid_settings_arr[ANGLE], &angle_pid_state);
  pid_reset(0, 0, &pid_settings_arr[VEL], &vel_pid_state);
  mahony_filter_init(&imuparams, 10.0f * MAHONY_FILTER_KP, MAHONY_FILTER_KI,
      2.0 * 2000.0f * M_PI / 180.0f, IMU_SAMPLE_TIME);

  wifi_setup();

  xTaskCreate(&httpd_task, "HTTP Daemon", 256, NULL, PRIO_COMMUNICATION, NULL);
  xTaskCreate(&main_loop, "Main loop", 256, NULL, PRIO_MAIN_LOOP, &xCalculationTask);
  xTaskCreate(&steering_watcher, "Steering watcher", 128, NULL, PRIO_MAIN_LOOP + 1, &xSteeringWatcher);
  xTaskCreate(&imu_watcher, "IMU watcher", 128, NULL, PRIO_MAIN_LOOP + 2, &xIMUWatcher);
  xTaskCreate(&maze_solver_task, "Maze solver", 256, NULL, PRIO_COMMUNICATION + 1, NULL);

  gpio_enable(IMU_INTERRUPT_PIN, GPIO_INPUT);
  gpio_set_interrupt(IMU_INTERRUPT_PIN, GPIO_INTTYPE_EDGE_POS, imu_interrupt_handler);
}
