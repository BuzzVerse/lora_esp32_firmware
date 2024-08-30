#ifndef _I2C_H_
#define _I2C_H_

#include "esp_err.h"
#include "driver/i2c.h"

/**
 * @brief Initialize the I2C master interface.
 *
 * @return
 *    - ESP_OK on success.
 *    - An error code on failure.
 */
esp_err_t i2c_init(void);

/**
 * @brief Write data to an I2C device.
 *
 * @param dev_addr I2C address of the device.
 * @param reg_addr Register address to write to.
 * @param data Pointer to the data buffer.
 * @param data_len Length of the data to write.
 *
 * @return
 *    - ESP_OK on success.
 *    - An error code on failure.
 */
esp_err_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len);

/**
 * @brief Read data from an I2C device.
 *
 * @param dev_addr I2C address of the device.
 * @param reg_addr Register address to read from.
 * @param data Pointer to the data buffer.
 * @param data_len Length of the data to read.
 *
 * @return
 *    - ESP_OK on success.
 *    - An error code on failure.
 */
esp_err_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len);

#endif // _I2C_H_
