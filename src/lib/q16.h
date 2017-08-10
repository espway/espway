/*
 * Fixed-point Q16.16 format calculations
 * Copyright (C) 2017  Sakari Kapanen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t q16;

#define Q16_ONE 65536
#define FLT_TO_Q16(x) ((q16)(Q16_ONE*(x)))
#define Q16_TO_INT(x) ((int)((x) >> 16))
#define INT_TO_Q16(X) (((q16)(x)) << 16)

q16 q16_div(q16 a, q16 b);
q16 q16_rsqrt(q16 x);

// Fixed point multiplication following
// https://github.com/PetteriAimonen/libfixmath/blob/master/libfixmath/fix16.c
inline q16 q16_mul(q16 x, q16 y)
{
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

inline q16 q16_exponential_smooth(q16 prevVal, q16 newVal, q16 alpha)
{
  return prevVal + q16_mul(alpha, newVal - prevVal);
}

#ifdef __cplusplus
}
#endif

