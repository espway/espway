#pragma once

#include <c_types.h>

typedef struct {
    uint8_t r, g, b;
} color_t;

void set_both_eyes(const color_t color);
void eyes_init(void);

