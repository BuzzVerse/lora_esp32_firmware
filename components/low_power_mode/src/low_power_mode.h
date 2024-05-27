#ifndef LOW_POWER_MODE_H
#define LOW_POWER_MODE_H

#include <stdint.h>

/**
 * @file low_power_mode.h
 * @authors Bykowski Olaf, Rafał Majewski
 *
 * @brief This file contains declarations for the Low Power Mode driver.
 *
 */

/**
 * @brief Function for making the device enter into the low power mode.
 *
 * @details This function makes the device enter into the low power mode. Before entering into the low power mode, make sure to set the sleep time using the @ref low_power_mode_set_sleep_time function.
 *
 * @warning All data in the RAM will be lost when the device enters into the low power mode.
 *
 */
void low_power_mode_enter_deep_sleep(void);

/**
 * @brief Function for setting the sleep time for the low power mode.
 *
 * @param sleep_time_sec The sleep time in seconds.
 *
 * @details This function sets the sleep time for the low power mode. The sleep time is specified in seconds.
 */
void low_power_mode_set_sleep_time(uint32_t sleep_time_sec);

#endif // LOW_POWER_MODE_H
