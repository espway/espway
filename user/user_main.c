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

#include <esp8266.h>
#include "uart.h"
#include "gpio.h"

#include "httpd.h"
#include "httpdespfs.h"
#include "espfs.h"
#include "webpages-espfs.h"
#include "cgiwebsocket.h"

#include "i2c_master.h"
#include "mpu6050.h"

const int LED_PIN = 2;
const int MPU_ADDR = 0x68;

#define N_TASKS_MAX 2

/* os_event_t gTaskQueue[N_TASKS_MAX]; */

mpuconfig gConfig = {
    .lowpass = 4,
    .sampleRateDivider = 1,
    .gyroRange = 0,
    .accelRange = 3,
    .enableInterrupt = true,
    .intActiveLow = false,
    .intOpenDrain = false,
    .beta = 0.2f
};
quaternion gQuat = { 1.0f, 0.0f, 0.0f, 0.0f };
int nSamples = 0;
unsigned long lastTime = 0;

void ICACHE_FLASH_ATTR compute(void) {
    int16_t buf[7];
    mpuReadIntStatus(MPU_ADDR);
    if (mpuReadRawData(MPU_ADDR, buf) != 0) return;
    mpuUpdateQuaternion(&gConfig, buf, &gQuat);
    float roll = rollAngle(&gQuat);
    float pitch = pitchAngle(&gQuat);

    printf("%d, %d\n",
        (int)(18000.0f / M_PI * pitch), (int)(18000.0f / M_PI * roll));

    nSamples += 1;
}

void ICACHE_FLASH_ATTR initAP(void) {
    char *ssid = "ESPway";
    struct softap_config conf;
    wifi_softap_get_config(&conf);

    memset(conf.ssid, 0, 32);
    memset(conf.password, 0, 64);
    memcpy(conf.ssid, ssid, strlen(ssid));
    conf.ssid_len = 0;
    conf.beacon_interval = 100;
    conf.max_connection = 1;
    conf.authmode = AUTH_OPEN;

    wifi_softap_set_config(&conf);
    wifi_set_opmode(SOFTAP_MODE);
}

void ICACHE_FLASH_ATTR socketReceive(Websock *ws, char *data, int len, int flags) {
    return;
}

void ICACHE_FLASH_ATTR socketConnect(Websock *ws) {
    ws->recvCb = socketReceive;
}

HttpdBuiltInUrl builtInUrls[] = {
    {"/", cgiRedirect, "/index.html"},
    {"/ws", cgiWebsocket, socketConnect},
    {"*", cgiEspFsHook, NULL},
    {NULL, NULL, NULL}
};

void ICACHE_FLASH_ATTR initHttpd(void) {
#ifdef ESPFS_POS
    espFsInit((void*)(0x40200000 + ESPFS_POS));
#else
    espFsInit((void*)(webpages_espfs_start));
#endif
    httpdInit(builtInUrls, 80);
}

void mpuInterrupt(uint32_t intr_mask, void *args) {
    uint32_t gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);

    /* system_os_post(USER_TASK_PRIO_2, 0, 0); */
}

void ICACHE_FLASH_ATTR user_init(void) {
    /* system_update_cpu_freq(SYS_CPU_160MHZ); */
    i2c_master_gpio_init();
    UART_SetBaudrate(UART0, 115200);

    initAP();
    initHttpd();

    mpuSetup(MPU_ADDR, &gConfig);

    GPIO_AS_OUTPUT(BIT12|BIT13|BIT14|BIT15);
    GPIO_OUTPUT(BIT12|BIT13|BIT14|BIT15, 0);

    gpio_intr_handler_register(mpuInterrupt, NULL);
    gpio_pin_intr_state_set(GPIO_ID_PIN(4), GPIO_PIN_INTR_POSEDGE);
    mpuReadIntStatus(MPU_ADDR);

    /* system_os_task(compute, USER_TASK_PRIO_2, gTaskQueue, N_TASKS_MAX); */
    /* system_os_post(USER_TASK_PRIO_2, 0, 0); */
    lastTime = system_get_time();
}

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

