#pragma once

#include <stdint.h>
#include <c_types.h>

#define Q16_MULTIPLIER 65536

typedef int32_t q16;

q16 q16_mul(q16 x, q16 y);
q16 q16_rsqrt(q16 x);
q16 float_to_q16(float f);
int16_t q16_to_int(q16 x);
q16 int_to_q16(int16_t x);

