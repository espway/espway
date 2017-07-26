#include "dnsresponder.h"

#include <espressif/esp_common.h>
#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
#include <dhcpserver.h>
#include <lwip/sockets.h>

/*
 * The functions dns_task and dnsresponder_init have been adapted from the
 * WifiCfg code by OurAirQuality.org with small modifications.
 * The license is included below.
 * Original source:
 * https://github.com/ourairquality/esp-open-rtos/blob/ourairquality/extras/wificfg/wificfg.c
 */

/*
 * WiFi configuration via a simple web server.
 *
 * Copyright (C) 2016 OurAirQuality.org
 *
 * Licensed under the Apache License, Version 2.0, January 2004 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *      http://www.apache.org/licenses/
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE CONTRIBUTORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS WITH THE SOFTWARE.
 *
 */

static ip4_addr_t server_addr;

static void dns_task(void *pvParameters)
{
    struct sockaddr_in serv_addr;
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(53);
    bind(fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    const struct ifreq ifreq0 = { "en0" };
    const struct ifreq ifreq1 = { "en1" };
    lwip_setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE,
                    sdk_wifi_get_opmode() == STATIONAP_MODE ? &ifreq1 : &ifreq0,
                    sizeof(ifreq0));

    for (;;) {
        char buffer[96];
        struct sockaddr src_addr;
        socklen_t src_addr_len = sizeof(src_addr);
        size_t count = recvfrom(fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&src_addr, &src_addr_len);

        /* Drop messages that are too large to send a response in the buffer */
        if (count > 0 && count <= sizeof(buffer) - 16 && src_addr.sa_family == AF_INET) {
            size_t qname_len = strlen(buffer + 12) + 1;
            uint32_t reply_len = 2 + 10 + qname_len + 16 + 4;

            char *head = buffer + 2;
            *head++ = 0x80; // Flags
            *head++ = 0x00;
            *head++ = 0x00; // Q count
            *head++ = 0x01;
            *head++ = 0x00; // A count
            *head++ = 0x01;
            *head++ = 0x00; // Auth count
            *head++ = 0x00;
            *head++ = 0x00; // Add count
            *head++ = 0x00;
            head += qname_len;
            *head++ = 0x00; // Q type
            *head++ = 0x01;
            *head++ = 0x00; // Q class
            *head++ = 0x01;
            *head++ = 0xC0; // LBL offs
            *head++ = 0x0C;
            *head++ = 0x00; // Type
            *head++ = 0x01;
            *head++ = 0x00; // Class
            *head++ = 0x01;
            *head++ = 0x00; // TTL
            *head++ = 0x00;
            *head++ = 0x00;
            *head++ = 0x78;
            *head++ = 0x00; // RD len
            *head++ = 0x04;
            *head++ = ip4_addr1(&server_addr);
            *head++ = ip4_addr2(&server_addr);
            *head++ = ip4_addr3(&server_addr);
            *head++ = ip4_addr4(&server_addr);

            sendto(fd, buffer, reply_len, 0, &src_addr, src_addr_len);
        }
    }
}

void dnsresponder_init(ip4_addr_t addr) {
    server_addr = addr;
    dhcpserver_set_router(&addr);
    dhcpserver_set_dns(&addr);
    xTaskCreate(dns_task, "WiFi Cfg DNS", 384, NULL, 2, NULL);
}

