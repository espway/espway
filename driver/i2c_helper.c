#include "ets_sys.h"

#include "i2c_helper.h"
#include "i2c_master.h"

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
    i2c_master_send_nack();
    return true;
}

uint8_t ICACHE_FLASH_ATTR i2c_master_readNextByte(bool ack) {
    uint8_t ret = i2c_master_readByte();
    i2c_master_setAck(!ack);
    return ret;
}

