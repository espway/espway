/*
 * Ultrasonic sensor driver for esp-open-rtos
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

#include "ultrasonic.h"
#include <espressif/esp_common.h>
#include <esp8266.h>
#include <esp/timer.h>
#include <esp/interrupts.h>
#include <esp/gpio.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <stdio.h>

typedef struct {
  uint8_t pin;
  uint32_t pin_mask;
  uint32_t rising_edge;
  uint32_t falling_edge;
  SemaphoreHandle_t sem;
} ultrasonic_sensor_t;

static ultrasonic_sensor_t sensors[ULTRASONIC_SENSOR_MAX_COUNT];
static int gpio_to_index[16];
static uint8_t sensor_count;

static void IRAM ultrasonic_irq_handler(uint8_t gpio_num)
{
  BaseType_t woken = pdFALSE;
  int i = gpio_to_index[gpio_num];
  if (i >= 0)
  {
    if (sensors[i].rising_edge == 0)
    {
      GPIO.CONF[sensors[i].pin] = SET_FIELD(GPIO.CONF[sensors[i].pin], GPIO_CONF_INTTYPE,
        GPIO_INTTYPE_EDGE_NEG);
      sensors[i].rising_edge = sdk_system_get_time();
    }
    else if (sensors[i].falling_edge == 0)
    {
      GPIO.CONF[sensors[i].pin] = SET_FIELD(GPIO.CONF[sensors[i].pin], GPIO_CONF_INTTYPE,
        GPIO_INTTYPE_NONE);
      sensors[i].falling_edge = sdk_system_get_time();
      xSemaphoreGiveFromISR(sensors[i].sem, &woken);
    }
  }

  portEND_SWITCHING_ISR(woken);
}

void ultrasonic_sensor_init(uint8_t pins[], uint8_t n_pins)
{
  if (n_pins > ULTRASONIC_SENSOR_MAX_COUNT)
  {
    printf("ultrasonic: too many sensors\n");
    return;
  }

  sensor_count = n_pins;

  for (uint8_t i = 0; i < 16; ++i)
    gpio_to_index[i] = -1;

  for (uint8_t i = 0; i < n_pins; ++i)
  {
    sensors[i].pin = pins[i];
    sensors[i].pin_mask = BIT(pins[i]);
    gpio_enable(pins[i], GPIO_INPUT);
    gpio_set_pullup(pins[i], true, false);
    gpio_to_index[pins[i]] = i;
    sensors[i].sem = xSemaphoreCreateBinary();
    if (sensors[i].sem == NULL)
    {
      printf("ultrasonic: failed to create semaphore\n");
      return;
    }
    gpio_set_interrupt(pins[i], GPIO_INTTYPE_EDGE_POS, ultrasonic_irq_handler);
    GPIO.CONF[pins[i]] = SET_FIELD(GPIO.CONF[pins[i]], GPIO_CONF_INTTYPE,
      GPIO_INTTYPE_NONE);
  }
}

int ultrasonic_sensor_read(uint8_t sensor_index)
{
  if (sensor_index >= sensor_count) return -1;

  ultrasonic_sensor_t* sensor = &sensors[sensor_index];

  xSemaphoreTake(sensor->sem, 0);

  // Emit trigger pulse
  sensor->rising_edge = 0;
  sensor->falling_edge = 0;

  {
    taskENTER_CRITICAL();
    GPIO.OUT_SET = sensor->pin_mask;
    sdk_os_delay_us(10);
    GPIO.OUT_CLEAR = sensor->pin_mask;
    // Listen to rising edge interrupts on the pin
    GPIO.ENABLE_OUT_CLEAR = sensor->pin_mask;
    GPIO.CONF[sensor->pin] = SET_FIELD(GPIO.CONF[sensor->pin],
      GPIO_CONF_INTTYPE, GPIO_INTTYPE_EDGE_POS);
    taskEXIT_CRITICAL();
  }

  int delta = -1;
  if (xSemaphoreTake(sensor->sem,
      ULTRASONIC_SENSOR_MAX_TIME_MS / portTICK_PERIOD_MS + 1))
    delta = sensor->falling_edge - sensor->rising_edge;

  GPIO.CONF[sensor->pin] = SET_FIELD(GPIO.CONF[sensor->pin], GPIO_CONF_INTTYPE,
    GPIO_INTTYPE_NONE);
  GPIO.ENABLE_OUT_SET = sensor->pin_mask;

  return delta;
}
