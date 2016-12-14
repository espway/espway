#pragma once

#include <stdint.h>
#include <c_types.h>

typedef int32_t q16;

q16 q16_mul(q16 x, q16 y);
q16 q16_rsqrt(q16 x);

