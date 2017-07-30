#pragma once

#include "q16.h"

typedef struct {
    q16 x, y, z;
} vector3d_fix;

inline vector3d_fix v3d_add(const vector3d_fix *u, const vector3d_fix *v) {
    vector3d_fix result = {
        u->x + v->x,
        u->y + v->y,
        u->z + v->z
    };
    return result;
}

inline vector3d_fix v3d_sub(const vector3d_fix *u, const vector3d_fix *v) {
    vector3d_fix result = {
        u->x - v->x,
        u->y - v->y,
        u->z - v->z
    };
    return result;
}

inline vector3d_fix v3d_mul(q16 a, const vector3d_fix *v) {
    vector3d_fix result = {
        q16_mul(a, v->x),
        q16_mul(a, v->y),
        q16_mul(a, v->z)
    };
    return result;
}

inline vector3d_fix v3d_cross(const vector3d_fix *u, const vector3d_fix *v) {
    vector3d_fix result =  {
        q16_mul(u->y, v->z) - q16_mul(u->z, v->y),
        q16_mul(u->z, v->x) - q16_mul(u->x, v->z),
        q16_mul(u->x, v->y) - q16_mul(u->y, v->x)
    };
    return result;
}

inline vector3d_fix v3d_normalize(const vector3d_fix *u) {
    q16 rnorm = q16_mul(u->x, u->x);
    rnorm += q16_mul(u->y, u->y);
    rnorm += q16_mul(u->z, u->z);
    rnorm = q16_rsqrt(rnorm);
    return v3d_mul(rnorm, u);
}
