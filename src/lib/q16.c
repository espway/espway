/*
 * A fixed point calculation utility library
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

#include "q16.h"
#include "q16_luts.h"

/*
 * q16_div is from libfixmath: https://github.com/PetteriAimonen/libfixmath/blob/master/libfixmath/fix16.c
 * MIT license: https://opensource.org/licenses/MIT
 */
q16 q16_div(q16 a, q16 b) {
  // This uses the basic binary restoring division algorithm.
  // It appears to be faster to do the whole division manually than
  // trying to compose a 64-bit divide out of 32-bit divisions on
  // platforms without hardware divide.

  if (b == 0)
    return 0x80000000;

  uint32_t remainder = (a >= 0) ? a : (-a);
  uint32_t divider = (b >= 0) ? b : (-b);

  uint32_t quotient = 0;
  uint32_t bit = 0x10000;

  /* The algorithm requires D >= R */
  while (divider < remainder)
  {
    divider <<= 1;
    bit <<= 1;
  }

  if (divider & 0x80000000)
  {
    // Perform one step manually to avoid overflows later.
    // We know that divider's bottom bit is 0 here.
    if (remainder >= divider)
    {
      quotient |= bit;
      remainder -= divider;
    }
    divider >>= 1;
    bit >>= 1;
  }

  /* Main division loop */
  while (bit && remainder)
  {
    if (remainder >= divider)
    {
      quotient |= bit;
      remainder -= divider;
    }

    remainder <<= 1;
    bit >>= 1;
  }	 

  q16 result = quotient;

  /* Figure out the sign of result */
  if ((a ^ b) & 0x80000000)
  {
    result = -result;
  }

  return result;
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
