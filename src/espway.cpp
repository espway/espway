extern "C" {
#include <string.h>
#include <espressif/esp_common.h>
#include <dhcpserver.h>
#include <FreeRTOS.h>
#include <task.h>
#include <httpd/httpd.h>
#include <esp8266.h>
#include <esp/uart.h>

#include "i2c/i2c.h"
#include "lib/mpu6050.h"
}


#define AP_SSID "ESPway"

void wifi_setup(void) {
    sdk_wifi_set_opmode(SOFTAP_MODE);
    struct ip_info ap_ip;
    IP4_ADDR(&ap_ip.ip, 192, 168, 4, 1);
    IP4_ADDR(&ap_ip.gw, 0, 0, 0, 0);
    IP4_ADDR(&ap_ip.netmask, 255, 255, 255, 0);
    sdk_wifi_set_ip_info(1, &ap_ip);

    struct sdk_softap_config ap_config = {};
    strcpy((char *)ap_config.ssid, AP_SSID);
    ap_config.channel = 3;
    ap_config.ssid_len = strlen(AP_SSID);
    ap_config.authmode = AUTH_OPEN;
    ap_config.max_connection = 1;
    ap_config.beacon_interval = 100;
    sdk_wifi_softap_set_config(&ap_config);

    ip_addr_t first_client_ip;
    IP4_ADDR(&first_client_ip, 192, 168, 4, 2);
    dhcpserver_start(&first_client_ip, 4);
}

void httpd_task(void *pvParameters)
{
    /* websocket_register_callbacks((tWsOpenHandler) websocket_open_cb, */
    /*         (tWsHandler) websocket_cb); */
    httpd_init();

    for (;;);
}

void do_loop(void *pvParameters) {
    int16_t raw_data[6];
    uint32_t time_old = 0;
    uint32_t time_new = 0;
    int n = 0;

    for (;;) {
        xTaskNotifyWait(0, 0, NULL, 1);
        mpu_read_raw_data(MPU_ADDR, raw_data);

        n += 1;
        if (n == 1024) {
            n = 0;
            time_new = sdk_system_get_time();
            uint32_t looptime = time_new - time_old;
            time_old = time_new;
        }
    }
}

TaskHandle_t xCalculationTask;

void mpu_interrupt_handler(uint8_t gpio_num) {
    BaseType_t xHigherPriorityTaskHasWoken = pdFALSE;
    xTaskNotifyFromISR(xCalculationTask, 0, eNoAction, &xHigherPriorityTaskHasWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskHasWoken);
}

extern "C" void user_init(void)
{
    uart_set_baud(0, 115200);
    wifi_setup();
    i2c_init(5, 0);
    mpu_init();

    xTaskCreate(&httpd_task, "HTTP Daemon", 128, NULL, 2, NULL);
    xTaskCreate(&do_loop, "Main loop", 256, NULL, 3, &xCalculationTask);

    gpio_enable(4, GPIO_INPUT);
    gpio_set_interrupt(4, GPIO_INTTYPE_EDGE_POS, mpu_interrupt_handler);
}

