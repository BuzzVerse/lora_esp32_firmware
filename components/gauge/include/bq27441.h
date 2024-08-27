#ifndef _BQ27441_H_
#define _BQ27441_H_

#include "esp_err.h"

/**
 * @file bq27441.h
 * @author Bykowski Olaf
 * @brief Functions to interact with the BQ27441 fuel gauge.
 */

/**
 * @brief Initialize the BQ27441 fuel gauge.
 *
 * This function unseals the device, enters configuration mode, sets the design capacity,
 * sets the terminate voltage, and then exits configuration mode.
 *
 * @return
 *    - ESP_OK on success.
 *    - An error code on failure.
 */
esp_err_t bq27441_init(void);

/**
 * @brief Set the design capacity of the battery.
 *
 * @param capacity The design capacity in mAh.
 *
 * @return
 *    - ESP_OK on success.
 *    - An error code on failure.
 */
esp_err_t bq27441_set_design_capacity(uint16_t capacity);

/**
 * @brief Read the design capacity of the battery.
 *
 * @param[out] capacity Pointer to store the design capacity in mAh.
 *
 * @return
 *    - ESP_OK on success.
 *    - An error code on failure.
 */
esp_err_t bq27441_read_design_capacity(uint16_t *capacity);

/**
 * @brief Read the state of charge (SoC) of the battery.
 *
 * @param[out] soc Pointer to store the state of charge in percentage.
 *
 * @return
 *    - ESP_OK on success.
 *    - An error code on failure.
 */
esp_err_t bq27441_read_soc(uint8_t *soc);

/**
 * @brief Read the voltage of the battery.
 *
 * @param[out] voltage Pointer to store the voltage in millivolts.
 *
 * @return
 *    - ESP_OK on success.
 *    - An error code on failure.
 */
esp_err_t bq27441_read_voltage(uint16_t *voltage);

#endif // _BQ27441_H_
