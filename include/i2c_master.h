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

#define I2C_MASTER_SDA_MUX PERIPHS_IO_MUX_GPIO0_U
#define I2C_MASTER_SCL_MUX PERIPHS_IO_MUX_GPIO5_U
#define I2C_MASTER_SDA_GPIO 0
#define I2C_MASTER_SCL_GPIO 5
#define I2C_MASTER_SDA_FUNC FUNC_GPIO0
#define I2C_MASTER_SCL_FUNC FUNC_GPIO5

#define SDA_HIGH() \
    gpio_output_conf(0, 0, 0, 1<<I2C_MASTER_SDA_GPIO)
#define SCL_HIGH() \
    gpio_output_conf(0, 0, 0, 1<<I2C_MASTER_SCL_GPIO)
#define SDA_LOW()  \
    gpio_output_conf(0, 1<<I2C_MASTER_SDA_GPIO, 1<<I2C_MASTER_SDA_GPIO, 0)
#define SCL_LOW()  \
    gpio_output_conf(0, 1<<I2C_MASTER_SCL_GPIO, 1<<I2C_MASTER_SCL_GPIO, 0)

#define SDA_HIGH_SCL_HIGH() \
    gpio_output_conf(0, 0, 0, (1<<I2C_MASTER_SCL_GPIO) | (1<<I2C_MASTER_SDA_GPIO))
#define SDA_HIGH_SCL_LOW() \
    gpio_output_conf(0, 1<<I2C_MASTER_SCL_GPIO, \
    1<<I2C_MASTER_SCL_GPIO, 1<<I2C_MASTER_SDA_GPIO)
#define SDA_LOW_SCL_HIGH() \
    gpio_output_conf(0, 1<<I2C_MASTER_SDA_GPIO, \
    1<<I2C_MASTER_SDA_GPIO, 1<<I2C_MASTER_SCL_GPIO)
#define SDA_LOW_SCL_LOW() \
    gpio_output_conf(0, (1<<I2C_MASTER_SDA_GPIO) | (1<<I2C_MASTER_SCL_GPIO), \
    (1<<I2C_MASTER_SDA_GPIO) | (1<<I2C_MASTER_SCL_GPIO), 0)

#define I2C_MASTER_DELAY 0

#define i2c_master_wait() os_delay_us(I2C_MASTER_DELAY)

void i2c_master_gpio_init(void);

void i2c_master_stop(void);
void i2c_master_start(void);
uint8_t i2c_master_readByte(void);
void i2c_master_writeByte(uint8_t wrdata);

bool i2c_master_checkAck(void);
void i2c_master_send_ack(bool);

bool i2c_master_transmitTo(uint8_t addr);
bool i2c_master_receiveFrom(uint8_t addr);
bool i2c_master_writeBytes(uint8_t *buf, int len);
bool i2c_master_readBytes(uint8_t *buf, int len);
uint8_t i2c_master_readNextByte(bool ack);

