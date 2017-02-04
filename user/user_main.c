/*
    Firmware for a segway-style robot using ESP8266.
    Copyright (C) 2016  Sakari Kapanen

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

#include "user_interface.h"
#include "osapi.h"
#include "espmissingincludes.h"
#include "gpio.h"
#include "driver/uart.h"

#include "i2c_master.h"

#define QUEUE_LEN 1

const int LED_PIN = 2;
const int MPU_ADDR = 0x68;

os_event_t gTaskQueue[QUEUE_LEN];

void ICACHE_FLASH_ATTR compute(os_event_t *e) {
    os_printf("compute called!\n");
    system_os_post(2, 0, 0);
}

void ICACHE_FLASH_ATTR user_init(void) {
    system_update_cpu_freq(80);
    i2c_master_gpio_init();
    uart_init(BIT_RATE_115200, BIT_RATE_115200);

    /*
    ETS_GPIO_INTR_DISABLE();

    gpio_output_set(0, BIT12|BIT13|BIT14|BIT15, BIT12|BIT13|BIT14|BIT15, 0);

    ETS_GPIO_INTR_ENABLE();
    */

    system_os_task(compute, 2, gTaskQueue, QUEUE_LEN);
    system_os_post(2, 0, 0);
}

