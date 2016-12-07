#pragma once

#include <stdint.h>

#define GET_ALIGNED_STRING_LEN(str) ((os_strlen(str) + 3) & ~3)
#define GET_ALIGNED_SIZE(s) (((s) + 3) & ~3)

void robotd_init(uint32_t port);
void robotd_init_ap(char *ssid);

