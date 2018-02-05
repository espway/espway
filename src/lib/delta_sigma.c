/*
 * Delta sigma modulator for ESP8266
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

#include <stdio.h>
#include <memory.h>
#include <esp/timer.h>
#include <esp/interrupts.h>
#include <esp/gpio.h>
#include <FreeRTOS.h>
#include <task.h>
#include "delta_sigma.h"

typedef struct {
  uint32_t gpio_mask;
  uint32_t duty;
  uint32_t acc;
} pin_state_t;

static pin_state_t g_pins[DELTA_SIGMA_MAX_PINS];
static uint8_t g_n_pins;
static uint32_t g_range;

static void IRAM timer_isr(void* arg)
{
  UBaseType_t flags = taskENTER_CRITICAL_FROM_ISR();
  uint32_t set_mask = 0;
  uint32_t clear_mask = 0;
  for (uint8_t i = 0; i < g_n_pins; ++i)
  {
    pin_state_t* pin = &g_pins[i];
    pin->acc += pin->duty;
    if (pin->acc >= g_range)
    {
      pin->acc -= g_range;
      set_mask |= pin->gpio_mask;
    }
    else
    {
      clear_mask |= pin->gpio_mask;
    }
  }
  GPIO.OUT_SET = set_mask;
  GPIO.OUT_CLEAR = clear_mask;
  taskEXIT_CRITICAL_FROM_ISR(flags);
}

void delta_sigma_set_duty(uint8_t channel, uint32_t duty)
{
  taskENTER_CRITICAL();
  if (duty > g_range) duty = g_range;
  g_pins[channel].duty = duty;
  taskEXIT_CRITICAL();
}

void delta_sigma_start(uint32_t period, uint32_t range, uint8_t pins[],
                       uint8_t n_pins)
{
  // Stop the timer during configuration
  timer_set_run(FRC1, false);

  g_n_pins = n_pins;
  for (uint8_t i = 0; i < n_pins; ++i)
  {
    g_pins[i].gpio_mask = BIT(pins[i]);
    g_pins[i].duty = 0;
    g_pins[i].acc = 0;
    gpio_enable(pins[i], GPIO_OUTPUT);
  }
  g_range = range;

  timer_set_interrupts(FRC1, false);
  timer_set_divider(FRC1, TIMER_CLKDIV_256);
  if (period > TIMER_FRC1_MAX_LOAD)
  {
    printf("delta_sigma: period %u too large\n", period);
    period = TIMER_FRC1_MAX_LOAD;
  }
  timer_set_load(FRC1, period);
  timer_set_reload(FRC1, true);
  _xt_isr_attach(INUM_TIMER_FRC1, timer_isr, NULL);
  timer_set_interrupts(FRC1, true);
  timer_set_run(FRC1, true);
}
