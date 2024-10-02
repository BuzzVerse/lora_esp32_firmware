#ifndef _BME280_H_
#define _BME280_H_

#include "bme280_defs.h"
#include "esp_err.h"
#include "sensor.h"

extern sensor_interface_t bme280_interface;
extern struct bme280_t bme280;

/**
 * @brief Function to initialize the BME280 sensor.
 *
 * @param sensor_config Pointer to the sensor configuration structure.
 *
 * @returns ESP_OK if successful, ESP_FAIL if unsuccessful
 */
esp_err_t bme280_init_driver(sensor_context_t *sensor_config);

/**
 * @brief Function to set oversampling settings.
 *
 * @param oversamp_pressure Pressure oversampling setting.
 * @param oversamp_temperature Temperature oversampling setting.
 * @param oversamp_humidity Humidity oversampling setting.
 *
 * @returns ESP_OK if successful, ESP_FAIL if unsuccessful.
 */
esp_err_t bme280_set_oversamp(bme280_oversampling_t oversamp_pressure, bme280_oversampling_t oversamp_temperature, bme280_oversampling_t oversamp_humidity);

/**
 * @brief Function to set standby time, filter coefficient, and power mode.
 *
 * @param standby_time Standby time setting.
 * @param filter_coeff Filter coefficient setting.
 * @param power_mode Power mode setting.
 *
 * @returns ESP_OK if successful, ESP_FAIL if unsuccessful.
 */
esp_err_t bme280_set_settings(bme280_standby_time_t standby_time, bme280_filter_coeff_t filter_coeff, bme280_power_mode_t power_mode);

/**
 * @brief Function to read pressure.
 *
 * @param pressure Pointer to the variable where the pressure will be stored.
 *
 * @returns ESP_OK if successful, ESP_FAIL if unsuccessful.
 */
esp_err_t bme280_read_pressure(double *pressure);

/**
 * @brief Function to read temperature.
 *
 * @param temperature Pointer to the variable where the temperature will be stored.
 *
 * @returns ESP_OK if successful, ESP_FAIL if unsuccessful.
 */
esp_err_t bme280_read_temperature(double *temperature);

/**
 * @brief Function to read humidity.
 *
 * @param humidity Pointer to the variable where the humidity will be stored.
 *
 * @returns ESP_OK if successful, ESP_FAIL if unsuccessful.
 */
esp_err_t bme280_read_humidity(double *humidity);

#endif // _BME280_H_
