#include "low_power_mode.h"
#include "esp_log.h"
#include <esp_sleep.h>

const static char *TAG = "low_power_mode_manager";

void low_power_mode_set_sleep_time(uint32_t sleep_time_sec)
{
    ESP_LOGI(TAG, "Enabling timer wakeup, %ld\n", sleep_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(sleep_time_sec * 1000000));
}

void low_power_mode_enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entering low power mode\n");
    esp_deep_sleep_start();
}