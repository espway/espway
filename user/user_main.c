#include <esp8266.h>
#include "driver/uart.h"

#include "httpd.h"
#include "httpdespfs.h"
#include "espfs.h"
#include "webpages-espfs.h"
#include "cgiwebsocket.h"

#include "i2c_master.h"
#include "i2c_helper.h"
#include "mpu6050.h"

const int LED_PIN = 2;

void initAP(void) {
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

void socketSendLedStatus(Websock *ws) {
    char status = GPIO_INPUT_GET(LED_PIN);
    cgiWebsocketSend(ws, &status, 1, WEBSOCK_FLAG_BIN);
}

void socketReceive(Websock *ws, char *data, int len, int flags) {
    if (len != 1) return;
    GPIO_OUTPUT_SET(LED_PIN, data[0]);
    socketSendLedStatus(ws);
}

void socketConnect(Websock *ws) {
    ws->recvCb = socketReceive;
    socketSendLedStatus(ws);
}

HttpdBuiltInUrl builtInUrls[] = {
    {"/", cgiRedirect, "/index.html"},
    {"/ws", cgiWebsocket, socketConnect},
    {"*", cgiEspFsHook, NULL},
    {NULL, NULL, NULL}
};

void initHttpd(void) {
#ifdef ESPFS_POS
    espFsInit((void*)(0x40200000 + ESPFS_POS));
#else
    espFsInit((void*)(webpages_espfs_start));
#endif
    httpdInit(builtInUrls, 80);
}

void user_init(void) {
    uart_init(BIT_RATE_115200, BIT_RATE_115200);
    gpio_init();
    i2c_master_gpio_init();

    initAP();
    initHttpd();

    /* mpuconfig mpuDefaultConfig = { */
    /*     .disableTemp = true, */
    /*     .lowpass = 3, */
    /*     .sampleRateDivider = 4, */
    /*     .gyroRange = 3, */
    /*     .accelRange = 0, */
    /*     .enableInterrupt = true */
    /* }; */
    /* int status = mpuSetup(0x68, &mpuDefaultConfig); */

    os_printf("\n");

    uint8_t addr = 0x68;
    i2c_master_start();
    i2c_master_writeByte(addr << 1);
    if (i2c_master_checkAck()) os_printf("ack 1\n");
    else os_printf("nack 1\n");

    i2c_master_writeByte(0x75);
    if (i2c_master_checkAck()) os_printf("ack 2\n");
    else os_printf("nack 2\n");

    i2c_master_start();
    i2c_master_writeByte((addr << 1) | 1);
    if (i2c_master_checkAck()) os_printf("ack 3\n");
    else os_printf("nack 3\n");
    uint8_t byte = i2c_master_readByte();
    i2c_master_send_nack();
    i2c_master_stop();

    os_printf("0x%02x\n", byte);
}

