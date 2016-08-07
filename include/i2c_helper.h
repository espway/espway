#ifndef I2C_HELPER
#define I2C_HELPER

bool i2c_master_transmitTo(uint8_t addr);
bool i2c_master_receiveFrom(uint8_t addr);
bool i2c_master_writeBytes(uint8_t *buf, int len);
bool i2c_master_readBytes(uint8_t *buf, int len);
uint8_t i2c_master_readNextByte(bool ack);

#endif
