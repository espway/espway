#ifndef __I2C_MASTER_H__
#define __I2C_MASTER_H__

#define I2C_MASTER_SDA_MUX PERIPHS_IO_MUX_GPIO4_U
#define I2C_MASTER_SCL_MUX PERIPHS_IO_MUX_GPIO5_U
#define I2C_MASTER_SDA_GPIO 4
#define I2C_MASTER_SCL_GPIO 5
#define I2C_MASTER_SDA_FUNC FUNC_GPIO4
#define I2C_MASTER_SCL_FUNC FUNC_GPIO5

#define I2C_MASTER_SDA_HIGH_SCL_HIGH()  \
    gpio_output_set(0, 0, 0, 1<<I2C_MASTER_SDA_GPIO | 1<<I2C_MASTER_SCL_GPIO)

#define I2C_MASTER_SDA_HIGH_SCL_LOW()  \
    gpio_output_set(0, 1<<I2C_MASTER_SCL_GPIO, 1<<I2C_MASTER_SCL_GPIO, 1<<I2C_MASTER_SDA_GPIO)

#define I2C_MASTER_SDA_LOW_SCL_HIGH()  \
    gpio_output_set(0, 1<<I2C_MASTER_SDA_GPIO, 1<<I2C_MASTER_SDA_GPIO, 1<<I2C_MASTER_SCL_GPIO)

#define I2C_MASTER_SDA_LOW_SCL_LOW()  \
    gpio_output_set(0, 1<<I2C_MASTER_SDA_GPIO | 1<<I2C_MASTER_SCL_GPIO, 1<<I2C_MASTER_SDA_GPIO | 1<<I2C_MASTER_SCL_GPIO, 0)

#ifndef I2C_MASTER_DELAY
#define I2C_MASTER_DELAY 5
#endif

#define i2c_master_wait os_delay_us
void i2c_master_gpio_init(void);

void i2c_master_stop(void);
void i2c_master_start(void);
void i2c_master_setAck(uint8 level);
uint8 i2c_master_getAck(void);
uint8 i2c_master_readByte(void);
void i2c_master_writeByte(uint8 wrdata);

bool i2c_master_checkAck(void);
void i2c_master_send_ack(void);
void i2c_master_send_nack(void);

#endif
