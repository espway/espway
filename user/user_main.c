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

mpufilter gMpuFilter;
int16_t gPitch;
int nSamples = 0;
unsigned long lastTime = 0;

void ICACHE_FLASH_ATTR compute(os_event_t *e) {
    int16_t buf[7];
    mpuReadIntStatus(MPU_ADDR);
    if (mpuReadRawData(MPU_ADDR, buf) != 0) return;
    mpuUpdatePitch(&gMpuFilter, buf, &gPitch);

    os_printf("%d\n", gPitch);

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

void socketSendLedStatus(Websock *ws) {
    char status = GPIO_INPUT_GET(GPIO_ID_PIN(LED_PIN));
    cgiWebsocketSend(ws, &status, 1, WEBSOCK_FLAG_BIN);
}

void socketReceive(Websock *ws, char *data, int len, int flags) {
    if (len != 1) return;
    GPIO_OUTPUT_SET(GPIO_ID_PIN(LED_PIN), data[0]);
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

    system_os_post(USER_TASK_PRIO_2, 0, 0);
}

void ICACHE_FLASH_ATTR user_init(void) {
    system_update_cpu_freq(SYS_CPU_160MHZ);
    gpio_init();
    i2c_master_gpio_init();
    uart_init(BIT_RATE_115200, BIT_RATE_115200);

    initAP();
    initHttpd();

    mpuconfig mpuConfig = {
        .disableTemp = true,
        .lowpass = 3,
        .sampleRateDivider = 0,
        .gyroRange = 0,
        .accelRange = 0,
        .enableInterrupt = true,
        .intActiveLow = false,
        .intOpenDrain = false
    };
    mpuSetup(MPU_ADDR, &mpuConfig);
    mpuSetupFilter(&mpuConfig, &gMpuFilter, 1);

    ETS_GPIO_INTR_DISABLE();
    ETS_GPIO_INTR_ATTACH(mpuInterrupt, NULL);
    gpio_pin_intr_state_set(GPIO_ID_PIN(4), GPIO_PIN_INTR_POSEDGE);
    mpuReadIntStatus(MPU_ADDR);

    system_os_task(compute, USER_TASK_PRIO_2, gTaskQueue, N_TASKS_MAX);
    system_os_post(USER_TASK_PRIO_2, 0, 0);
    lastTime = system_get_time();
    os_timer_setfn(&gReportTimer, reportSamplerate, NULL);
    os_timer_arm(&gReportTimer, 500, true);

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);
    gpio_output_set(0, BIT12|BIT13|BIT14|BIT15, BIT12|BIT13|BIT14|BIT15, 0);

    ETS_GPIO_INTR_ENABLE();
}

