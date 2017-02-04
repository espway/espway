#include "q16.h"
#include "q16_luts.h"

// Fixed point multiplication following
// https://github.com/PetteriAimonen/libfixmath/blob/master/libfixmath/fix16.c
q16 q16_mul(q16 x, q16 y) {
    int32_t xhi = x >> 16, yhi = y >> 16;
    uint32_t xlo = x & 0xffff, ylo = y & 0xffff;

    int32_t hi = xhi*yhi;
    int32_t mid = xhi*ylo + yhi*xlo;
    uint32_t lo = xlo*ylo;
    hi += mid >> 16;
    uint32_t prod_lo = lo + (mid << 16);
    if (prod_lo < lo) hi += 1;

    return (hi << 16) | (prod_lo >> 16);
}

q16 q16_div(q16 x, q16 y) {
    int32_t res = x / y;
    return res << 16;
}

q16 q16_rsqrt(q16 x) {
    int power = 14 - (__builtin_clz(x) & ~0x01);

    q16 x_normalized = x >> (power + 10);

    size_t i = x_normalized - 64;
    q16 y = Q16_RSQRT_LUT[i] << (-(power / 2) + 1);
    q16 y2 = q16_mul(y, y);
    y = q16_mul(y, 3*Q16_ONE - q16_mul(x, y2)) / 2;
    y2 = q16_mul(y, y);
    y = q16_mul(y, 3*Q16_ONE - q16_mul(x, y2)) / 2;

    return y;
}

q16 q16_exponential_smooth(q16 prevVal, q16 newVal, q16 alpha) {
    return prevVal + q16_mul(alpha, newVal - prevVal);
}

