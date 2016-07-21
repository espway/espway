#include <esp8266.h>
#include "httpd.h"
#include "httpdespfs.h"
#include "espfs.h"
#include "webpages-espfs.h"
#include "captdns.h"

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

HttpdBuiltInUrl builtInUrls[] = {
    {"*", cgiRedirectApClientToHostname, "espway.robot"},
    {"/", cgiRedirect, "/index.html"},
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
    captdnsInit();
}

void user_init(void) {
    initAP();
    initHttpd();
}

void user_rf_pre_init() {
}
