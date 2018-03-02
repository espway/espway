/*
 * Firmware for a segway-style robot using ESP8266.
 * Copyright (C) 2018  Sakari Kapanen
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

#include "espway.h"
#include <vector>

extern "C" {
#include "lib/ultrasonic.h"
#include "lib/samplebuffer.h"
#include "lib/pid.h"
#include "lib/eyes.h"
#include <stdio.h>
#include <stdlib.h>
}

static bool probe_for_sensors(const std::vector<uint8_t>& sensors)
{
  const unsigned int PROBING_TIME_MS = 5000;
  const unsigned int PROBING_INTERVAL_MS = 100;
  const unsigned int PROBE_COUNT = PROBING_TIME_MS / PROBING_INTERVAL_MS;

  std::vector<bool> sensor_found(sensors.size(), false);
  for (size_t i = 0; i < PROBE_COUNT; ++i)
  {
    for (size_t j = 0; j < sensors.size(); ++j)
    {
      if (!sensor_found.at(j) && ultrasonic_sensor_read(sensors.at(j)) > 0)
        sensor_found.at(j) = true;
    }

    bool all_found = true;
    for (bool found : sensor_found)
      if (found == false)
      {
        all_found = false;
        break;
      }
    if (all_found) return true;

    vTaskDelay(PROBING_INTERVAL_MS / portTICK_PERIOD_MS);
  }

  return false;
}

void maze_solver_task(void *pvParameters)
{
  std::vector<uint8_t> pins {ULTRASONIC_SENSOR_SIDE_GPIO,
    ULTRASONIC_SENSOR_FRONT_GPIO};
  ultrasonic_sensor_init(&pins[0], pins.size());

  samplebuffer_t* side_buffer = samplebuffer_init(5);
  samplebuffer_t* front_buffer = samplebuffer_init(5);

  pidsettings pid;
  pidstate pid_state;
  pid_coeffs coeffs = { FLT_TO_Q16(0.0003f), 0, FLT_TO_Q16(0.00002f) };
  pid_initialize(&coeffs, FLT_TO_Q16(0.01f), FLT_TO_Q16(-0.3f), FLT_TO_Q16(0.3f),
    false, &pid);

  q16 speed = FLT_TO_Q16(0.4f);
  q16 ref_distance = CM_TO_US(10) * Q16_ONE;

  if (!probe_for_sensors({0, 1}))
  {
    set_both_eyes(RED);
    goto exit;
  }

  for (;;)
  {
    pid_reset(ref_distance, 0, &pid, &pid_state);
    set_steering(0, 0);
    samplebuffer_reset(side_buffer, INT32_MAX);
    samplebuffer_reset(front_buffer, INT32_MAX);

    while (get_state() != RUNNING) vTaskDelay(500 / portTICK_PERIOD_MS);

    q16 actual_speed = 0;
    while (get_state() == RUNNING)
    {
      {
        int side_value = ultrasonic_sensor_read(0);
        samplebuffer_add_sample(side_buffer, side_value);
      }
      {
        int front_value = ultrasonic_sensor_read(1);
        samplebuffer_add_sample(front_buffer, front_value);
      }

      {
        q16 bias = 0;
        int front_median = samplebuffer_median(front_buffer);
        int side_median = samplebuffer_median(side_buffer);
        const int WALL_AVOIDANCE = CM_TO_US(15);

        if (front_median > 0 && front_median < WALL_AVOIDANCE)
          bias = q16_mul((front_median - WALL_AVOIDANCE) * Q16_ONE, FLT_TO_Q16(0.0008f));
        else if (side_median > 0 && side_median < CM_TO_US(15))
          bias = pid_compute(side_median * Q16_ONE, ref_distance, &pid, &pid_state);
        else
          bias = FLT_TO_Q16(0.12f);

        actual_speed = q16_exponential_smooth(actual_speed, speed, FLT_TO_Q16(0.05f));
        set_steering(actual_speed, bias);
      }

      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }

exit:
  free(front_buffer);
  free(side_buffer);
  vTaskDelete(NULL);
}

