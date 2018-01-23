/*
 * Fixed-point Q16.16 format vector calculations
 * Copyright (C) 2017  Sakari Kapanen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "q16.h"

typedef union {
  struct {
    q16 x, y, z;
  };
  q16 data[3];
} vector3d_fix;

static inline vector3d_fix v3d_add(const vector3d_fix *u, const vector3d_fix *v)
{
  vector3d_fix result = {{
    u->x + v->x,
    u->y + v->y,
    u->z + v->z
  }};
  return result;
}

static inline vector3d_fix v3d_sub(const vector3d_fix *u, const vector3d_fix *v)
{
  vector3d_fix result = {{
    u->x - v->x,
    u->y - v->y,
    u->z - v->z
  }};
  return result;
}

static inline vector3d_fix v3d_mul(q16 a, const vector3d_fix *v)
{
  vector3d_fix result = {{
    q16_mul(a, v->x),
    q16_mul(a, v->y),
    q16_mul(a, v->z)
  }};
  return result;
}

static inline vector3d_fix v3d_cross(const vector3d_fix *u, const vector3d_fix *v)
{
  vector3d_fix result =  {{
    q16_mul(u->y, v->z) - q16_mul(u->z, v->y),
    q16_mul(u->z, v->x) - q16_mul(u->x, v->z),
    q16_mul(u->x, v->y) - q16_mul(u->y, v->x)
  }};
  return result;
}

static inline vector3d_fix v3d_normalize(const vector3d_fix *u)
{
  q16 rnorm = q16_mul(u->x, u->x);
  rnorm += q16_mul(u->y, u->y);
  rnorm += q16_mul(u->z, u->z);
  rnorm = q16_rsqrt(rnorm);
  return v3d_mul(rnorm, u);
}
