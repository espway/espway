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
#include "user_interface.h"
#include "driver/uart.h"
#include "gpio.h"

#include "httpd.h"
#include "httpdespfs.h"
#include "espfs.h"
#include "webpages-espfs.h"
#include "cgiwebsocket.h"

#include "i2c_master.h"
#include "mpu6050.h"

#define QUEUE_LEN 1

const int LED_PIN = 2;
const int MPU_ADDR = 0x68;

mpuconfig gConfig = {
    .lowpass = 3,
    .sampleRateDivider = 1,
    .gyroRange = 3,
    .accelRange = 0,
    .enableInterrupt = true,
    .intActiveLow = false,
    .intOpenDrain = false,
    .beta = 0.05f
};
quaternion gQuat = { 1.0f, 0.0f, 0.0f, 0.0f };
int nSamples = 0;
unsigned long lastTime = 0;
const unsigned long gQuatUpdateInterval = 100000;
unsigned long gLastQuatUpdate = 0;

os_event_t gTaskQueue[QUEUE_LEN];

void ICACHE_FLASH_ATTR compute(os_event_t *e) {
    int16_t buf[6];
    mpuReadIntStatus(MPU_ADDR);
    if (mpuReadRawData(MPU_ADDR, buf) != 0) return;
    mpuUpdateQuaternion(&gConfig, buf, &gQuat);
    float roll = rollAngle(&gQuat);
    float pitch = pitchAngle(&gQuat);

    /* os_printf("%d, %d\n", (int)(18000.0f / M_PI * pitch), (int)(18000.0f / M_PI * roll)); */
    float scale = 1000.0f;
    int16_t qbuf[] = {
        gQuat.q0 * scale,
        gQuat.q1 * scale,
        gQuat.q2 * scale,
        gQuat.q3 * scale
    };

    unsigned long time = system_get_time();
    if (time - gLastQuatUpdate > gQuatUpdateInterval) {
        cgiWebsockBroadcast("/ws", (uint8_t *)qbuf, 8, WEBSOCK_FLAG_BIN);
        gLastQuatUpdate = time;
    }

    nSamples += 1;
    if (nSamples == 1000) {
        /* os_printf("%lu\n", nSamples * 1000000UL / (time - lastTime)); */
        lastTime = time;
        nSamples = 0;
    }
}

void ICACHE_FLASH_ATTR initAP(void) {
    char *ssid = "ESPway";
    struct softap_config conf;
    wifi_softap_get_config(&conf);

    os_memset(conf.ssid, 0, 32);
    os_memset(conf.password, 0, 64);
    os_memcpy(conf.ssid, ssid, strlen(ssid));
    conf.ssid_len = 0;
    conf.beacon_interval = 100;
    conf.max_connection = 2;
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

void mpuInterrupt(uint32_t mask, void *args) {
    uint32_t gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);

    system_os_post(2, 0, 0);
}

void ICACHE_FLASH_ATTR user_init(void) {
    system_update_cpu_freq(80);
    i2c_master_gpio_init();
    UART_SetBaudrate(UART0, 115200);

    initAP();
    initHttpd();

    if (mpuSetup(MPU_ADDR, &gConfig) != 0) {
        os_printf("MPU config failed!\n");
    }

    ETS_GPIO_INTR_DISABLE();

    gpio_output_set(0, BIT12|BIT13|BIT14|BIT15, BIT12|BIT13|BIT14|BIT15, 0);

    ETS_GPIO_INTR_ATTACH(mpuInterrupt, NULL);
    gpio_pin_intr_state_set(4, GPIO_PIN_INTR_POSEDGE);
    mpuReadIntStatus(MPU_ADDR);

    ETS_GPIO_INTR_ENABLE();

    lastTime = system_get_time();

    system_os_task(compute, 2, gTaskQueue, QUEUE_LEN);
}

