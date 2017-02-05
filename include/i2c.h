/*
    I2C protocol implementation for ESP8266.
    Copyright (C) 2016  Sakari Kapanen

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

#include <stdint.h>
#include <stdbool.h>

#include <eagle_soc.h>

#define I2C_SDA_MUX PERIPHS_IO_MUX_GPIO4_U
#define I2C_SCL_MUX PERIPHS_IO_MUX_GPIO5_U
#define I2C_SDA_GPIO 4
#define I2C_SCL_GPIO 5
#define I2C_SDA_FUNC FUNC_GPIO4
#define I2C_SCL_FUNC FUNC_GPIO5

#define SDA_HIGH() \
    GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << I2C_SDA_GPIO)
#define SCL_HIGH() \
    GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << I2C_SCL_GPIO)
#define SDA_LOW()  \
    GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << I2C_SDA_GPIO)
#define SCL_LOW()  \
    GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << I2C_SCL_GPIO)
#define SDA_READ() \
    ((GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << I2C_SDA_GPIO)) != 0)
#define SCL_READ() \
    ((GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << I2C_SCL_GPIO)) != 0)

#define I2C_WAIT_COUNT 1
#define I2C_CLOCK_STRETCH_MAX 10

void i2c_gpio_init(void);

void i2c_stop(void);
void i2c_start(void);
uint8_t i2c_read_byte(void);
void i2c_write_byte(uint8_t wrdata);

bool i2c_check_ack(void);
void i2c_send_ack(bool);

bool i2c_transmit_to(uint8_t addr);
bool i2c_receive_from(uint8_t addr);
bool i2c_write_bytes(uint8_t *buf, int len);
bool i2c_read_bytes(uint8_t *buf, int len);

