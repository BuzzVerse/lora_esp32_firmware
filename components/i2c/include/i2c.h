#ifndef I2C_H
#define I2C_H

#include "esp_err.h"

esp_err_t i2c_init(void);
esp_err_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len);
esp_err_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len);
void delay_ms(uint32_t ticks);

#endif // I2C_H
