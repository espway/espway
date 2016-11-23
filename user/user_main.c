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
#include "driver/uart.h"

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

os_event_t gTaskQueue[N_TASKS_MAX];
os_timer_t gReportTimer;

mpuconfig gConfig = {
    .disableTemp = true,
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

void ICACHE_FLASH_ATTR compute(os_event_t *e) {
    int16_t buf[7];
    mpuReadIntStatus(MPU_ADDR);
    if (mpuReadRawData(MPU_ADDR, buf) != 0) return;
    mpuUpdateQuaternion(&gConfig, buf, &gQuat);
    vector3 g;
    gravityVector(&gQuat, &g);
    double roll = rollAngle(&g);
    double pitch = pitchAngle(&g);

    os_printf("%d, %d\n",
        (int)(18000.0f / M_PI * pitch), (int)(18000.0f / M_PI * roll));

    nSamples += 1;
}

void ICACHE_FLASH_ATTR reportSamplerate(void *args) {
    unsigned long curTime = system_get_time();
    unsigned long sampleRate = (1000000UL * (uint32_t)nSamples) /
        (curTime - lastTime);
    lastTime = curTime;
    nSamples = 0;
    /* os_printf("%lu\n", sampleRate); */
}

void ICACHE_FLASH_ATTR initAP(void) {
    char *ssid = "esp8266";
    struct softap_config conf;
    wifi_softap_get_config(&conf);

    os_memset(conf.ssid, 0, 32);
    os_memset(conf.password, 0, 64);
    os_memcpy(conf.ssid, ssid, strlen(ssid));
    conf.ssid_len = 0;
    conf.beacon_interval = 100;
    conf.max_connection = 4;
    conf.authmode = AUTH_OPEN;

    wifi_softap_set_config(&conf);
    wifi_set_opmode(SOFTAP_MODE);
}

void ICACHE_FLASH_ATTR socketSendLedStatus(Websock *ws) {
    char status = GPIO_INPUT_GET(GPIO_ID_PIN(LED_PIN));
    cgiWebsocketSend(ws, &status, 1, WEBSOCK_FLAG_BIN);
}

void ICACHE_FLASH_ATTR socketReceive(Websock *ws, char *data, int len, int flags) {
    if (len != 1) return;
    GPIO_OUTPUT_SET(GPIO_ID_PIN(LED_PIN), data[0]);
    socketSendLedStatus(ws);
}

void ICACHE_FLASH_ATTR socketConnect(Websock *ws) {
    ws->recvCb = socketReceive;
    socketSendLedStatus(ws);
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

    system_os_post(USER_TASK_PRIO_2, 0, 0);
}

void ICACHE_FLASH_ATTR user_init(void) {
    system_update_cpu_freq(SYS_CPU_160MHZ);
    gpio_init();
    i2c_master_gpio_init();
    uart_init(BIT_RATE_115200, BIT_RATE_115200);

    initAP();
    initHttpd();

    mpuSetup(MPU_ADDR, &gConfig);

    ETS_GPIO_INTR_DISABLE();

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);
    gpio_output_set(0, BIT12|BIT13|BIT14|BIT15, BIT12|BIT13|BIT14|BIT15, 0);

    ETS_GPIO_INTR_ATTACH(mpuInterrupt, NULL);
    gpio_pin_intr_state_set(GPIO_ID_PIN(4), GPIO_PIN_INTR_POSEDGE);
    mpuReadIntStatus(MPU_ADDR);

    system_os_task(compute, USER_TASK_PRIO_2, gTaskQueue, N_TASKS_MAX);
    system_os_post(USER_TASK_PRIO_2, 0, 0);
    lastTime = system_get_time();
    os_timer_setfn(&gReportTimer, reportSamplerate, NULL);
    os_timer_arm(&gReportTimer, 500, true);

    ETS_GPIO_INTR_ENABLE();
}

