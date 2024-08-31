#ifndef _BME280_DEFS_H_
#define _BME280_DEFS_H_

#include "bme280_lib.h"
#include <stdint.h>

/**
 * @file bme280_defs.h
 * @authors Olaf Bykowski, Rafał Majewski
 *
 * @brief This file contains definitions for the BME280 driver.
 */

/**
 * @brief Configuration structure for BQ27441.
 */
typedef struct
{
    uint8_t i2c_address;
} bme280_config_t;

/**
 * @brief Oversampling values for the BME280 sensor.
 *
 * @details Oversampling is used to reduce noise in the output data. Higher oversampling values result in less noise.
 *
 * The BME280 sensor supports the following oversampling values:
 */
typedef enum
{
    OVERSAMP_SKIPPED = BME280_OVERSAMP_SKIPPED, /**< Oversampling skipped*/
    OVERSAMP_1X = BME280_OVERSAMP_1X,           /**< Oversampling 1x*/
    OVERSAMP_2X = BME280_OVERSAMP_2X,           /**< Oversampling 2x*/
    OVERSAMP_4X = BME280_OVERSAMP_4X,           /**< Oversampling 4x*/
    OVERSAMP_8X = BME280_OVERSAMP_8X,           /**< Oversampling 8x*/
    OVERSAMP_16X = BME280_OVERSAMP_16X          /**< Oversampling 16x*/
} bme280_oversampling_t;

/**
 * @brief Standby time values for the BME280 sensor.
 *
 * @details Standby time is the time between two consecutive measurements.
 *
 * The BME280 sensor supports the following standby time values:
 */
typedef enum
{
    STANDBY_1MS = BME280_STANDBY_TIME_1_MS,      /**< Standby time 1ms*/
    STANDBY_10MS = BME280_STANDBY_TIME_10_MS,    /**< Standby time 10ms*/
    STANDBY_20MS = BME280_STANDBY_TIME_20_MS,    /**< Standby time 20ms*/
    STANDBY_63MS = BME280_STANDBY_TIME_63_MS,    /**< Standby time 63ms*/
    STANDBY_125MS = BME280_STANDBY_TIME_125_MS,  /**< Standby time 125ms*/
    STANDBY_250MS = BME280_STANDBY_TIME_250_MS,  /**< Standby time 250ms*/
    STANDBY_500MS = BME280_STANDBY_TIME_500_MS,  /**< Standby time 500ms*/
    STANDBY_1000MS = BME280_STANDBY_TIME_1000_MS /**< Standby time 1000ms*/
} bme280_standby_time_t;

/**
 * @brief Filter coefficient values for the BME280 sensor.
 *
 * @details Filter coefficient is used to reduce noise in the output data. Higher filter coefficient values result in less noise.
 *
 * The BME280 sensor supports the following filter coefficient values:
 */
typedef enum
{
    FILTER_OFF = BME280_FILTER_COEFF_OFF, /**< Filter coefficient off*/
    FILTER_2 = BME280_FILTER_COEFF_2,     /**< Filter coefficient 2*/
    FILTER_4 = BME280_FILTER_COEFF_4,     /**< Filter coefficient 4*/
    FILTER_8 = BME280_FILTER_COEFF_8,     /**< Filter coefficient 8*/
    FILTER_16 = BME280_FILTER_COEFF_16    /**< Filter coefficient 16*/
} bme280_filter_coeff_t;

/**
 * @brief Power mode values for the BME280 sensor.
 *
 * @details Power mode is used to set the sensor to sleep, forced or normal mode.
 *
 * The BME280 sensor supports the following power mode values:
 */
typedef enum
{
    MODE_SLEEP = BME280_SLEEP_MODE,   /**< Sleep mode*/
    MODE_FORCED = BME280_FORCED_MODE, /**< Forced mode*/
    MODE_NORMAL = BME280_NORMAL_MODE  /**< Normal mode*/
} bme280_power_mode_t;

#endif // _BME280_DEFS_H_
