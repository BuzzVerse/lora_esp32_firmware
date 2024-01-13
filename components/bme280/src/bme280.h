#ifndef BME280_H
#define BME280_H

/**
 * @file bme280.h
 * @authors Bykowski Olaf, Rafał Majewski
 *
 *  @{
 */

esp_err_t bme280_init_driver(uint8_t dev_addr);

esp_err_t bme280_set_oversamp(void);

esp_err_t bme280_set_settings(void);

double bme280_read_pressure(void);

double bme280_read_temperature(void);

double bme280_read_humidity(void);

#endif // BME280_H

/** @} */
