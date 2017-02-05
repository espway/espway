#include "eyes.h"

#include "ws2812_i2s.h"

void ICACHE_FLASH_ATTR set_both_eyes(const color_t color) {
    uint8_t buf[] = {
        color.g, color.r, color.b,
        color.g, color.r, color.b
    };
    ws2812_push(buf, 6);
}

void ICACHE_FLASH_ATTR eyes_init(void) {
    ws2812_init();
}

