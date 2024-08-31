#ifndef _I2C_H_
#define _I2C_H_

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "sensor.h"

/**
 * @brief Initialize the I2C master interface.
 *
 * @return
 *    - ESP_OK on success.
 *    - An error code on failure.
 */
esp_err_t i2c_init(void);

/**
 * @brief Initialize an I2C device.
 *
 * @param dev_config Pointer to the device configuration.
 * @param dev_handle Pointer to the device handle.
 *
 * @return
 *    - ESP_OK on success.
 *    - An error code on failure.
 */
esp_err_t i2c_device_init(i2c_device_config_t* dev_config, i2c_master_dev_handle_t* dev_handle);

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
esp_err_t i2c_write_custom(sensor_interface_t *intf, uint8_t reg_addr, uint8_t *data, size_t data_len);

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
esp_err_t i2c_read_custom(sensor_interface_t *intf, uint8_t reg_addr, uint8_t *data, size_t data_len);

#endif // _I2C_H_
