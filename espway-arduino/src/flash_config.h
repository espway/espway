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

#define MAX_CONFIG_SIZE 1024

#include <EEPROM.h>

void flash_config_begin() {
    EEPROM.begin(MAX_CONFIG_SIZE);
}

template<typename T>
bool read_flash_config(T &config, uint32_t version) {
    uint32_t flash_version = 0;
    // First check that the config version matches
    EEPROM.get<uint32_t>(0, flash_version);
    if (flash_version != version) {
        return false;
    }

    // Read the actual config data
    EEPROM.get<T>(sizeof(uint32_t), config);
    return true;
}

template<typename T>
bool write_flash_config(const T &config, uint32_t version) {
    EEPROM.put<uint32_t>(0, version);
    EEPROM.put<T>(sizeof(uint32_t), config);
    return EEPROM.commit();
}

bool clear_flash_config() {
    EEPROM.write(0, 0);
    return EEPROM.commit();
}

