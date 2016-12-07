#pragma once

#include <stdint.h>

#define GET_ALIGN_STRING_LEN(str) ((os_strlen(str) + 3) & ~3)

void robotd_init(uint32_t port);
void robotd_init_ap(char *ssid);

