#include "low_power_mode.h"
#include <esp_sleep.h>

void low_power_mode_set_sleep_time(int sleep_time_sec)
{
    printf("Enabling timer wakeup, %ds\n", sleep_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(sleep_time_sec * 1000000));
}

void low_power_mode_enter_deep_sleep(void)
{
    printf("Entering low power mode\n");
    low_power_mode_set_sleep_time(CONFIG_LOW_POWER_MODE_SLEEP_TIME_SEC);
    esp_deep_sleep_start();
}