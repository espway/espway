/*
    Utility functions for reading/writing versioned config on the flash memory
    of ESP8266
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

#pragma once

#define CONFIG_SECTOR 0x7d
#define MAX_CONFIG_SIZE 1024

#define ALIGN_FOUR_BYTES(addr) (((addr) + 3) & ~3)

bool read_flash_config(void * config, size_t config_size, uint32_t version);
bool write_flash_config(void * config, size_t config_size, uint32_t version);
bool clear_flash_config(void);

