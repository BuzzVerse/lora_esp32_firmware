#ifndef LOW_POWER_MODE_H
#define LOW_POWER_MODE_H

/**
 * @brief Initialize low power mode
 */
void low_power_mode_enter_deep_sleep(void);

/**
 * @brief Set the time to sleep in low power mode
 */
void low_power_mode_set_sleep_time(int sleep_time_sec);

#endif // LOW_POWER_MODE_H
