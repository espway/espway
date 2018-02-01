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
#include <stdio.h>

void maze_solver_task(void *pvParameters)
{
  uint8_t pins[] = {ULTRASONIC_SENSOR_SIDE_GPIO};
  ultrasonic_sensor_init(pins, sizeof(pins) / sizeof(pins[0]));

  for (;;)
  {
    int value = ultrasonic_sensor_read(0);
    printf("delay = %d\n", value);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
