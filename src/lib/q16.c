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
q16 q16_div(q16 a, q16 b)
{
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

q16 q16_rsqrt(q16 x)
{
  /* A Q16.16 fixed point approximation of the reciprocal square root.
   *
   * Let the number x be represented as follows:
   *   x = m * 2^2n,
   * where m is real in [1,4[, and n is integer. Then
   *   1 / sqrt(x) = 1 / sqrt(m * 2^n) = 1 / (sqrt(m) * 2^n)
   * = 1 / sqrt(m) * 2^-n.
   *
   * Thus it is sufficient to use a look-up table of 1 / sqrt(m) values for
   * m in the interval [1, 4[. The power calculations in fixed point are easily
   * implemented using shifts.
   */

  /* To calculate the (even) power, we count leading zeros, clear the odd bit
   * and subtract the value from 14. E.g. the Q16.16 representation of number 2
   * is 1 << 17 and has thus 17 trailing and 14 leading zeros.
   */
  int power = 14 - (__builtin_clz(x) & ~0x01);

  /* The look-up table is indexed such that indices for [1,4[ are in the
   * interval [0, 192[. Hence we normalize x to that interval by first
   * dividing by 2^2n, then shifting right by 10 (thus 1<<17 becomes 1<<7) to
   * bring it to the range [0, 256[ and finally subtract 64 to bring it down to
   * the correct index range.*/
  size_t i = (x >> (power + 10)) - 64;

  /* The Q16.16 values of 1 / sqrt(m) in the LUT are scaled by 0.5, so we
   * cancel that here by adding 1 to the power.
   */
  q16 y = Q16_RSQRT_LUT[i] << (-(power / 2) + 1);

  /* So far we have obtained a good estimate. Let's improve it a little by some
   * iteration. We are solving the unknown y from the equation
   *   y = 1/sqrt(x),
   * or equivalently,
   *   f(y) = 1 / y^2 - x = 0.
   * Differentiating by y, we get
   *   f'(y) = -2 / y^3.
   * Then the Newton-Raphson iteration can be applied as follows:
   *   y_(n+1) = y_n - f(y) / f'(y)
   *           = y_n - (1 / y_n^2 - x) / (-2 / y_n^3)
   *           = y_n - (y_n - y_n^3 * x) / (-2)
   *           = y_n * (3/2 - y_n^2 * x)
   * 
   * The equivalent form is used to avoid division in the iteration equation.
   */
  q16 y2 = q16_mul(y, y);
  y = q16_mul(y, 3*Q16_ONE - q16_mul(x, y2)) / 2;
  y2 = q16_mul(y, y);
  y = q16_mul(y, 3*Q16_ONE - q16_mul(x, y2)) / 2;

  return y;
}

