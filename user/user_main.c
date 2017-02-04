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

#include <user_interface.h>
#include <osapi.h>
#include <espmissingincludes.h>
#include <gpio.h>
#include <driver/uart.h>

#include <esp8266.h>
#include "httpd.h"
#include "cgiflash.h"
#include "httpdespfs.h"
#include "espfs.h"
#include "webpages-espfs.h"

#include "i2c_master.h"

#define QUEUE_LEN 1

const int LED_PIN = 2;
const int MPU_ADDR = 0x68;

os_event_t gTaskQueue[QUEUE_LEN];

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
    {"/flash/upload", cgiUploadFirmware, &uploadParams},
    {"/flash/reboot", cgiRebootFirmware, NULL},
#endif

    {"/", cgiRedirect, "/index.html"},
    {"*", cgiEspFsHook, NULL}, //Catch-all cgi function for the filesystem
    {NULL, NULL, NULL}
};

void ICACHE_FLASH_ATTR compute(os_event_t *e) {
    os_printf("compute called!\n");
    system_os_post(2, 0, 0);
}

void ICACHE_FLASH_ATTR wifi_init(void) {
    struct softap_config config;
    wifi_set_opmode_current(0x02);
    wifi_softap_get_config(&config); // Get config first.
    os_memset(config.ssid, 0, 32);
    os_memset(config.password, 0, 64);
    os_memcpy(config.ssid, "ESPway", 6);
    config.authmode = AUTH_OPEN;
    config.ssid_len = 0;// or its actual length
    config.beacon_interval = 100;
    config.max_connection = 1; // how many stations can connect to ESP8266 softAP at most.
    wifi_softap_set_config(&config);// Set ESP8266 softap config .
}

void ICACHE_FLASH_ATTR user_init(void) {
    system_update_cpu_freq(80);
    i2c_master_gpio_init();
    uart_init(BIT_RATE_115200, BIT_RATE_115200);

    wifi_init();

    // 0x40200000 is the base address for spi flash memory mapping, ESPFS_POS is the position
    // where image is written in flash that is defined in Makefile.
#ifdef ESPFS_POS
    espFsInit((void*)(0x40200000 + ESPFS_POS));
#else
    espFsInit((void*)(webpages_espfs_start));
#endif
    httpdInit(builtInUrls, 80);

    /*
       ETS_GPIO_INTR_DISABLE();

       gpio_output_set(0, BIT12|BIT13|BIT14|BIT15, BIT12|BIT13|BIT14|BIT15, 0);

       ETS_GPIO_INTR_ENABLE();
       */

    system_os_task(compute, 2, gTaskQueue, QUEUE_LEN);
    system_os_post(2, 0, 0);
}

