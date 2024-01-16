#ifndef BME280_H
#define BME280_H
#include "esp_err.h"

/**
 * @file bme280.h
 * @authors Bykowski Olaf, Rafał Majewski
 *
 * @brief This file contains the declaration for the BME280 driver.
 * @addtogroup bme280 BME 280 Driver
 *
 *  @{
 */

/**
 * @brief Function to initialize the BME280 sensor.
 *
 * @param dev_addr I2C address of the BME280 sensor. Takes a hex value.
 *
 * @details This function initializes the I2C bus with values from config and initializes the BME280 sensor.
 *
 */
esp_err_t bme280_init_driver(uint8_t dev_addr);

/**
 * @brief Function to set oversampling settings
 *
 * @param oversamp_pressure Pressure oversampling setting
 *
 * @param oversamp_temperature Temperature oversampling setting
 *
 * @param oversamp_humidity Humidity oversampling setting
 *
 * @details This function sets the oversampling settings for the BME280 sensor.
 *
 */
esp_err_t bme280_set_oversamp(uint8_t oversamp_pressure, uint8_t oversamp_temperature, uint8_t oversamp_humidity);

/**
 * @brief Function to set standby time, filter coefficient and power mode
 *
 * @param standby_time Standby time setting
 *
 * @param filter_coeff Filter coefficient setting
 *
 * @param power_mode Power mode setting
 *
 * @details This function sets the standby time, filter coefficient and power mode for the BME280 sensor.
 *
 */
esp_err_t bme280_set_settings(uint8_t standby_time, uint8_t filter_coeff, uint8_t power_mode);

/**
 * @brief Function to read pressure
 *
 * @return Pressure in Pa
 *
 * @details This function reads the pressure from the BME280 sensor.
 *
 */
double bme280_read_pressure(void);

/**
 * @brief Function to read temperature
 *
 * @return Temperature in C
 *
 * @details This function reads the temperature from the BME280 sensor.
 *
 */
double bme280_read_temperature(void);

/**
 * @brief Function to read humidity
 *
 * @return Humidity in %
 *
 * @details This function reads the humidity from the BME280 sensor.
 *
 */
double bme280_read_humidity(void);

#endif // BME280_H

/** @} */
