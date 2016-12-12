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
#include "driver/uart.h"
#include "gpio.h"

#include "i2c_master.h"
#include "mpu6050.h"
#include "robotd.h"

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
bool gSendQuat = false;
unsigned long gLastSentQuat = 0;
const unsigned long QUAT_INTERVAL = 50000;
const float QUAT_SCALE = 1000.0f;
int16_t buf[6];

os_event_t gTaskQueue[QUEUE_LEN];

void ICACHE_FLASH_ATTR compute(os_event_t *e) {
    mpuReadIntStatus(MPU_ADDR);
    if (mpuReadRawData(MPU_ADDR, buf) != 0) return;
    mpuUpdateQuaternion(&gConfig, buf, &gQuat);
    float pitch = pitchAngle(&gQuat);

    unsigned long time = system_get_time();
    if (gSendQuat && time - gLastSentQuat > QUAT_INTERVAL) {
        int16_t qdata[] = {
            QUAT_SCALE * gQuat.q0,
            QUAT_SCALE * gQuat.q1,
            QUAT_SCALE * gQuat.q2,
            QUAT_SCALE * gQuat.q3
        };
        robotd_websocket_send_all(WS_OPCODE_BIN, (char *)qdata, 8);
        gSendQuat = false;
        gLastSentQuat = time;
    }
}

void mpuInterrupt(uint32_t mask, void *args) {
    uint32_t gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);

    system_os_post(2, 0, 0);
}

void ICACHE_FLASH_ATTR
websocketCb(robotd_client *pclient, uint8_t opcode, char *data, size_t length) {
    gSendQuat = true;
}

void ICACHE_FLASH_ATTR user_init(void) {
    system_update_cpu_freq(80);
    i2c_master_gpio_init();
    UART_SetBaudrate(UART0, 115200);

    robotd_set_websocket_receive_cb(websocketCb);
    robotd_init_ap("ESPway");
    robotd_init(80);

    if (mpuSetup(MPU_ADDR, &gConfig) != 0) {
        os_printf("MPU config failed!\n");
    }

    ETS_GPIO_INTR_DISABLE();

    gpio_output_set(0, BIT12|BIT13|BIT14|BIT15, BIT12|BIT13|BIT14|BIT15, 0);

    ETS_GPIO_INTR_ATTACH(mpuInterrupt, NULL);
    gpio_pin_intr_state_set(4, GPIO_PIN_INTR_POSEDGE);
    mpuReadIntStatus(MPU_ADDR);

    ETS_GPIO_INTR_ENABLE();

    system_os_task(compute, 2, gTaskQueue, QUEUE_LEN);
}

