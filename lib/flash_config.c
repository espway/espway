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

#include <esp8266.h>

#include "flash_config.h"

#define ALIGN_FOUR_BYTES(addr) (((addr) + 3) & ~3)

static uint32_t config_buf[MAX_CONFIG_SIZE];

bool ICACHE_FLASH_ATTR read_flash_config(void * config, size_t config_size,
    uint32_t version) {
    size_t aligned_size = ALIGN_FOUR_BYTES(config_size);
    uint32_t flash_version;
    // First check that the config version matches
    bool success = system_param_load(CONFIG_SECTOR, 0, &flash_version, 4);
    if (!success || flash_version != version) {
        return false;
    }

    // Read the actual config data
    if (system_param_load(CONFIG_SECTOR, 4, config_buf, aligned_size)) {
        os_memcpy(config, config_buf, aligned_size);
        return true;
    }
    return false;
}

bool ICACHE_FLASH_ATTR write_flash_config(void * config, size_t config_size,
    uint32_t version) {
    size_t aligned_size = ALIGN_FOUR_BYTES(config_size);
    config_buf[0] = version;
    os_memcpy(&config_buf[1], config, config_size);
    return system_param_save_with_protect(CONFIG_SECTOR, config_buf,
        aligned_size + 4);
}

bool ICACHE_FLASH_ATTR clear_flash_config(void) {
    uint32_t zero_version = 0;
    return system_param_save_with_protect(CONFIG_SECTOR, &zero_version, 4);
}

