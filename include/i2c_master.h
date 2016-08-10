#ifndef __I2C_MASTER_H__
#define __I2C_MASTER_H__

#define I2C_MASTER_SDA_MUX PERIPHS_IO_MUX_GPIO4_U
#define I2C_MASTER_SCL_MUX PERIPHS_IO_MUX_GPIO5_U
#define I2C_MASTER_SDA_GPIO 4
#define I2C_MASTER_SCL_GPIO 5
#define I2C_MASTER_SDA_FUNC FUNC_GPIO4
#define I2C_MASTER_SCL_FUNC FUNC_GPIO5

#define SDA_HIGH() \
    gpio_output_set(0, 0, 0, 1<<I2C_MASTER_SDA_GPIO)
#define SCL_HIGH() \
    gpio_output_set(0, 0, 0, 1<<I2C_MASTER_SCL_GPIO)
#define SDA_LOW()  \
    gpio_output_set(0, 1<<I2C_MASTER_SDA_GPIO, 1<<I2C_MASTER_SDA_GPIO, 0)
#define SCL_LOW()  \
    gpio_output_set(0, 1<<I2C_MASTER_SCL_GPIO, 1<<I2C_MASTER_SCL_GPIO, 0)

#define SDA_HIGH_SCL_HIGH() \
    gpio_output_set(0, 0, 0, (1<<I2C_MASTER_SCL_GPIO) | (1<<I2C_MASTER_SDA_GPIO))
#define SDA_HIGH_SCL_LOW() \
    gpio_output_set(0, 1<<I2C_MASTER_SCL_GPIO, \
    1<<I2C_MASTER_SCL_GPIO, 1<<I2C_MASTER_SDA_GPIO)
#define SDA_LOW_SCL_HIGH() \
    gpio_output_set(0, 1<<I2C_MASTER_SDA_GPIO, \
    1<<I2C_MASTER_SDA_GPIO, 1<<I2C_MASTER_SCL_GPIO)
#define SDA_LOW_SCL_LOW() \
    gpio_output_set(0, (1<<I2C_MASTER_SDA_GPIO) | (1<<I2C_MASTER_SCL_GPIO), \
    (1<<I2C_MASTER_SDA_GPIO) | (1<<I2C_MASTER_SCL_GPIO), 0)

#define I2C_MASTER_DELAY 0

#define i2c_master_wait() os_delay_us(I2C_MASTER_DELAY)

void i2c_master_gpio_init(void);

void i2c_master_stop(void);
void i2c_master_start(void);
uint8 i2c_master_readByte(void);
void i2c_master_writeByte(uint8 wrdata);

bool i2c_master_checkAck(void);
void i2c_master_send_ack(bool);

bool i2c_master_transmitTo(uint8_t addr);
bool i2c_master_receiveFrom(uint8_t addr);
bool i2c_master_writeBytes(uint8_t *buf, int len);
bool i2c_master_readBytes(uint8_t *buf, int len);
uint8_t i2c_master_readNextByte(bool ack);

#endif
