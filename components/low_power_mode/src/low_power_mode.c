#include "low_power_mode.h"
#include "esp_log.h"
#include <esp_sleep.h>

const static char *TAG = "low_power_mode_manager";

/**
 * This function sets the sleep time for the low power mode. The sleep time is specified in seconds.
 *
 * @param sleep_time_sec The sleep time in seconds.
 */
void low_power_mode_set_sleep_time(int sleep_time_sec)
{
    ESP_LOGI(TAG, "Enabling timer wakeup, %ds\n", sleep_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(sleep_time_sec * 1000000));
}

/**
 * This function makes the device enter into the low power mode. Before entering into the low power mode, it sets the sleep time using the `low_power_mode_set_sleep_time` function.
 *
 * @warning All data in the RAM will be lost when the device enters into the low power mode.
 */
void low_power_mode_enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entering low power mode\n");
    esp_deep_sleep_start();
}