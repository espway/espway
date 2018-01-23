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
  } components;
  q16 data[3];
} vector3d_fix;

static inline vector3d_fix v3d_add(const vector3d_fix *u, const vector3d_fix *v)
{
  vector3d_fix result = {{
    u->components.x + v->components.x,
    u->components.y + v->components.y,
    u->components.z + v->components.z
  }};
  return result;
}

static inline vector3d_fix v3d_sub(const vector3d_fix *u, const vector3d_fix *v)
{
  vector3d_fix result = {{
    u->components.x - v->components.x,
    u->components.y - v->components.y,
    u->components.z - v->components.z
  }};
  return result;
}

static inline vector3d_fix v3d_mul(q16 a, const vector3d_fix *v)
{
  vector3d_fix result = {{
    q16_mul(a, v->components.x),
    q16_mul(a, v->components.y),
    q16_mul(a, v->components.z)
  }};
  return result;
}

static inline vector3d_fix v3d_cross(const vector3d_fix *u, const vector3d_fix *v)
{
  vector3d_fix result =  {{
    q16_mul(u->components.y, v->components.z) - q16_mul(u->components.z, v->components.y),
    q16_mul(u->components.z, v->components.x) - q16_mul(u->components.x, v->components.z),
    q16_mul(u->components.x, v->components.y) - q16_mul(u->components.y, v->components.x)
  }};
  return result;
}

static inline vector3d_fix v3d_normalize(const vector3d_fix *u)
{
  q16 rnorm = q16_mul(u->components.x, u->components.x);
  rnorm += q16_mul(u->components.y, u->components.y);
  rnorm += q16_mul(u->components.z, u->components.z);
  rnorm = q16_rsqrt(rnorm);
  return v3d_mul(rnorm, u);
}
