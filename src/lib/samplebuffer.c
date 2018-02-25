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

#include "samplebuffer.h"
#include <stdlib.h>
#include <assert.h>

samplebuffer_t* samplebuffer_init(size_t sample_count)
{
  assert(sample_count > 0);

  // Reserve a temporary buffer for sorting samples
  samplebuffer_t* buf = (samplebuffer_t*)malloc(sizeof(samplebuffer_t) + (size_t)2*sample_count*sizeof(int32_t));
  if (!buf) return NULL;

  buf->sample_count = sample_count;
  buf->next_index = 0;
  buf->middle1 = sample_count / 2;
  buf->middle2 = (sample_count - 1) / 2;

  return buf;
}

void samplebuffer_add_sample(samplebuffer_t* buffer, int32_t sample)
{
  buffer->buffer[buffer->next_index] = sample;
  buffer->next_index = (buffer->next_index + 1) % buffer->sample_count;
}

void samplebuffer_reset(samplebuffer_t* buffer, int32_t value)
{
  for (size_t i = 0; i < buffer->sample_count; ++i)
    buffer->buffer[i] = value;
  buffer->next_index = 0;
}

// Performs an insertion sort of the samples and calculates the median
// from there.
int32_t samplebuffer_median(samplebuffer_t* buffer)
{
  const int32_t* data = buffer->buffer;
  int32_t* tmp = &buffer->buffer[buffer->sample_count];

  for (size_t i = 0; i < buffer->sample_count; ++i)
  {
    int32_t value = data[i];

    // Find the insertion point
    size_t j = 0;
    while (j < i && value >= tmp[j]) ++j;
    // Insertion point found, shift elements right
    for (size_t k = i; k > j; --k)
      tmp[k] = tmp[k - 1];

    // Insert the value
    tmp[j] = value;
  }

  // Pick the median
  return (tmp[buffer->middle1] + tmp[buffer->middle2]) / 2;
}

