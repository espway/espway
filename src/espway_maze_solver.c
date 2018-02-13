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
#include "lib/ultrasonic.h"
#include "lib/samplebuffer.h"
#include "lib/pid.h"
#include "lib/eyes.h"
#include <stdio.h>
#include <stdlib.h>

void maze_solver_task(void *pvParameters)
{
  uint8_t pins[] = {ULTRASONIC_SENSOR_SIDE_GPIO};
  ultrasonic_sensor_init(pins, sizeof(pins) / sizeof(pins[0]));

  samplebuffer_t* buffer = samplebuffer_init(5);

  pidsettings pid;
  pidstate pid_state;
  pid_coeffs coeffs = { FLT_TO_Q16(0.0005f), 0, 0 };
  pid_initialize(&coeffs, FLT_TO_Q16(0.01f), FLT_TO_Q16(-0.3f), FLT_TO_Q16(0.3f),
    false, &pid);

  q16 speed = FLT_TO_Q16(0.35f);
  q16 ref_distance = 580 * Q16_ONE;

  for (;;)
  {
    pid_reset(ref_distance, 0, &pid, &pid_state);
    q16 bias = 0;

    while (get_state() == RUNNING)
    {
      int value = ultrasonic_sensor_read(0);

      if (value > 0)
      {
        samplebuffer_add_sample(buffer, value);
        int median = samplebuffer_median(buffer);

        if (median > 1500) bias = 0;
        else bias = pid_compute(median * Q16_ONE, ref_distance, &pid, &pid_state);


        printf("median = %5d, bias = %5d\n", median, bias);
      }

      q16 color = 128 * bias / Q16_ONE;
      color_t clr;
      clr.red = color;
      clr.green = 0;
      clr.blue = 0;
      set_both_eyes(clr);

      set_steering(speed, bias);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  free(buffer);
}

