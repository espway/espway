/*
 * Ringbuffer with median of samples calculation for small sample counts
 * Copyright (C) 2018  Sakari Kapanen
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

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct {
  size_t sample_count;
  size_t next_index;
  size_t middle1;
  size_t middle2;
  int32_t buffer[];
} samplebuffer_t;

// Allocate and initialize ring buffer. Free it with free()
samplebuffer_t* samplebuffer_init(size_t sample_count);
void samplebuffer_add_sample(samplebuffer_t* buffer, int32_t value);
void samplebuffer_reset(samplebuffer_t* buffer, int32_t value);

// Performs an insertion sort of the samples and calculates the median
// from there.
int32_t samplebuffer_median(samplebuffer_t* buffer);

