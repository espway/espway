/*
 * lwIP and FreeRTOS RAII lock helpers
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

#include <FreeRTOS.h>
#include <semphr.h>
#include <lwip/tcpip.h>

struct MutexLock
{
  MutexLock(SemaphoreHandle_t handle)
  {
    xSemaphoreTake(handle, portMAX_DELAY);
    m_handle = handle;
  }

  ~MutexLock()
  {
    xSemaphoreGive(m_handle);
  }

  private:
    SemaphoreHandle_t m_handle;
};

struct LwipCoreLock
{
  LwipCoreLock()
  {
    LOCK_TCPIP_CORE();
  }

  ~LwipCoreLock()
  {
    UNLOCK_TCPIP_CORE();
  }
};

