/*
    Fixed-point Q16.16 format calculations
    Copyright (C) 2017  Sakari Kapanen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "q16.h"
#include "q16_luts.h"

// Fixed point multiplication following
// https://github.com/PetteriAimonen/libfixmath/blob/master/libfixmath/fix16.c
q16 ICACHE_FLASH_ATTR q16_mul(q16 x, q16 y) {
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

q16 ICACHE_FLASH_ATTR q16_div(q16 x, q16 y) {
    int32_t res = x / y;
    return res << 16;
}

q16 ICACHE_FLASH_ATTR q16_rsqrt(q16 x) {
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

q16 ICACHE_FLASH_ATTR q16_exponential_smooth(q16 prevVal, q16 newVal, q16 alpha) {
    return prevVal + q16_mul(alpha, newVal - prevVal);
}

