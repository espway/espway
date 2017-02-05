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

#include <ets_sys.h>
#include <osapi.h>
#include <os_type.h>
#include <espmissingincludes.h>
#include <gpio.h>
#include <eagle_soc.h>

#include "i2c.h"

static inline void i2c_wait() {
    unsigned int i = I2C_WAIT_COUNT;
    while (i--) {
        asm volatile("nop");
    }
}

void ICACHE_FLASH_ATTR
i2c_gpio_init(void)
{
    ETS_GPIO_INTR_DISABLE();

    PIN_FUNC_SELECT(I2C_SDA_MUX, I2C_SDA_FUNC);
    PIN_FUNC_SELECT(I2C_SCL_MUX, I2C_SCL_FUNC);

    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << I2C_SDA_GPIO);
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << I2C_SCL_GPIO);

    SDA_HIGH();
    SCL_HIGH();

    ETS_GPIO_INTR_DISABLE();
}

void ICACHE_FLASH_ATTR
i2c_start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    i2c_wait();
    SDA_LOW();
    i2c_wait();
}

void ICACHE_FLASH_ATTR
i2c_stop(void)
{
    uint32_t i = 0;
    SDA_LOW();
    SCL_LOW();
    i2c_wait();
    SCL_HIGH();
    while(SCL_READ() == 0 && (i++) < I2C_CLOCK_STRETCH_MAX);
    SDA_HIGH();
    i2c_wait();
}

static void ICACHE_FLASH_ATTR
i2c_write_bit(bool bit)
{
    uint32_t i = 0;
    SCL_LOW();
    if (bit) {
        SDA_HIGH();
    } else {
        SDA_LOW();
    }
    i2c_wait();
    SCL_HIGH();
    while(SCL_READ() == 0 && (i++) < I2C_CLOCK_STRETCH_MAX);
    i2c_wait();
}

static bool ICACHE_FLASH_ATTR
i2c_read_bit()
{
    uint32_t i = 0;
    bool bit;
    SCL_LOW();
    SDA_HIGH();
    i2c_wait();
    SCL_HIGH();
    while(SCL_READ() == 0 && (i++) < I2C_CLOCK_STRETCH_MAX);
    bit = SDA_READ();
    SCL_LOW();
    i2c_wait();
    return bit;
}

bool ICACHE_FLASH_ATTR
i2c_check_ack(void)
{
    return i2c_read_bit() ? false : true;
}

void ICACHE_FLASH_ATTR
i2c_send_ack(bool ack)
{
    i2c_write_bit(ack ? 0x0 : 0x1);
}

uint8_t ICACHE_FLASH_ATTR
i2c_read_byte(void)
{
    uint8_t retVal = 0;

    for (int i = 7; i >= 0; i--) {
        retVal |= i2c_read_bit() << i;
    }

    return retVal;
}

void ICACHE_FLASH_ATTR
i2c_write_byte(uint8_t wrdata)
{
    for (int i = 7; i >= 0; i--) {
        i2c_write_bit((wrdata >> i) & 0x1);
    }
}

bool ICACHE_FLASH_ATTR i2c_transmit_to(uint8_t addr) {
    i2c_write_byte(addr << 1);
    return i2c_check_ack();
}

bool ICACHE_FLASH_ATTR i2c_receive_from(uint8_t addr) {
    i2c_write_byte((addr << 1) | 1);
    return i2c_check_ack();
}

bool ICACHE_FLASH_ATTR i2c_write_bytes(uint8_t *buf, int len) {
    for (int i = 0; i < len; ++i) {
        i2c_write_byte(buf[i]);
        if (i2c_check_ack() == 0) return false;
    }
    return true;
}

bool ICACHE_FLASH_ATTR i2c_read_bytes(uint8_t *buf, int len) {
    int i;
    for (i = 0; i < len - 1; ++i) {
        buf[i] = i2c_read_byte();
        i2c_send_ack(true);
    }
    buf[i] = i2c_read_byte();
    i2c_send_ack(false);
    return true;
}

