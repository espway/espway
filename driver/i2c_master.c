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

#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"

#include "i2c_master.h"

void ICACHE_FLASH_ATTR
i2c_master_gpio_init(void)
{
    ETS_GPIO_INTR_DISABLE() ;

    PIN_FUNC_SELECT(I2C_MASTER_SDA_MUX, I2C_MASTER_SDA_FUNC);
    PIN_FUNC_SELECT(I2C_MASTER_SCL_MUX, I2C_MASTER_SCL_FUNC);

    GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO)),
        GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO))) |
        GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE));
    GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SCL_GPIO)),
        GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SCL_GPIO))) |
        GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE));

    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) |
        (1 << I2C_MASTER_SDA_GPIO) | (1 << I2C_MASTER_SDA_GPIO));

    PIN_PULLUP_EN(I2C_MASTER_SDA_MUX);
    PIN_PULLUP_EN(I2C_MASTER_SCL_MUX);

    SDA_HIGH_SCL_HIGH();

    ETS_GPIO_INTR_ENABLE() ;
}

void ICACHE_FLASH_ATTR
i2c_master_start(void)
{
    SDA_HIGH_SCL_HIGH();
    i2c_master_wait();
    SDA_LOW();
    i2c_master_wait();
}

void ICACHE_FLASH_ATTR
i2c_master_stop(void)
{
    SDA_LOW_SCL_LOW();
    i2c_master_wait();
    SCL_HIGH();
    i2c_master_wait();
    SDA_HIGH();
    i2c_master_wait();
}

LOCAL void ICACHE_FLASH_ATTR
i2c_master_write_bit(bool bit)
{
    if (bit) SDA_HIGH_SCL_LOW();
    else SDA_LOW_SCL_LOW();
    i2c_master_wait();
    SCL_HIGH();
    i2c_master_wait();
}

LOCAL bool ICACHE_FLASH_ATTR
i2c_master_read_bit()
{
    bool bit;
    SDA_HIGH_SCL_LOW();
    i2c_master_wait();
    SCL_HIGH();
    i2c_master_wait();
    bit = GPIO_INPUT_GET(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO));
    SCL_LOW();
    i2c_master_wait();
    return bit;
}

bool ICACHE_FLASH_ATTR
i2c_master_checkAck(void)
{
    return i2c_master_read_bit() ? false : true;
}

void ICACHE_FLASH_ATTR
i2c_master_send_ack(bool ack)
{
    i2c_master_write_bit(ack ? 0x0 : 0x1);
}

uint8 ICACHE_FLASH_ATTR
i2c_master_readByte(void)
{
    uint8 retVal = 0;

    for (int8_t i = 7; i >= 0; i--) {
        retVal |= i2c_master_read_bit() << i;
    }

    return retVal;
}

void ICACHE_FLASH_ATTR
i2c_master_writeByte(uint8 wrdata)
{
    for (int8_t i = 7; i >= 0; i--) {
        i2c_master_write_bit((wrdata >> i) & 0x1);
    }
}

bool ICACHE_FLASH_ATTR i2c_master_transmitTo(uint8_t addr) {
    i2c_master_writeByte(addr << 1);
    return i2c_master_checkAck();
}

bool ICACHE_FLASH_ATTR i2c_master_receiveFrom(uint8_t addr) {
    i2c_master_writeByte((addr << 1) | 1);
    return i2c_master_checkAck();
}

bool ICACHE_FLASH_ATTR i2c_master_writeBytes(uint8_t *buf, int len) {
    for (int i = 0; i < len; i++) {
        i2c_master_writeByte(buf[i]);
        if (i2c_master_checkAck() == 0) return false;
    }
    return true;
}

bool ICACHE_FLASH_ATTR i2c_master_readBytes(uint8_t *buf, int len) {
    int i;
    for (i = 0; i < len - 1; i++) {
        buf[i] = i2c_master_readNextByte(true);
    }
    buf[i] = i2c_master_readNextByte(false);
    i2c_master_send_ack(false);
    return true;
}

uint8_t ICACHE_FLASH_ATTR i2c_master_readNextByte(bool ack) {
    uint8_t ret = i2c_master_readByte();
    i2c_master_send_ack(ack);
    return ret;
}

