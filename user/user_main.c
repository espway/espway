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
const int MPU_ADDR = 0x68;

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

void mpuInterrupt(uint32_t intr_mask, void *args) {
    uint32_t gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);

    mpuReadIntStatus(MPU_ADDR);

    int16_t buf[7];
    mpuReadRawData(MPU_ADDR, buf);

    os_printf("%d\n", buf[0]);
}

void user_init(void) {
    gpio_init();
    i2c_master_gpio_init();
    uart_init(BIT_RATE_115200, BIT_RATE_115200);

    initAP();
    initHttpd();

    mpuconfig mpuDefaultConfig = {
        .disableTemp = true,
        .lowpass = 3,
        .sampleRateDivider = 4,
        .gyroRange = 3,
        .accelRange = 0,
        .enableInterrupt = true,
        .intActiveLow = true,
        .intOpenDrain = true
    };
    int status = mpuSetup(MPU_ADDR, &mpuDefaultConfig);

    ETS_GPIO_INTR_ATTACH(mpuInterrupt, NULL);
    gpio_pin_intr_state_set(GPIO_ID_PIN(0), GPIO_PIN_INTR_NEGEDGE);
    mpuReadIntStatus(MPU_ADDR);
}

