#include <stdio.h>
#include "low_power_mode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief The task that enters into the low power mode.
 * 
 * FreeRTOS task that enters into the low power mode. It sets the sleep time using the `low_power_mode_set_sleep_time` function and then enters into the low power mode using the `low_power_mode_enter_deep_sleep` function.
 */
void low_power_mode_task()
{
    low_power_mode_set_sleep_time(CONFIG_LOW_POWER_MODE_SLEEP_TIME_SEC);
    for (;;)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        low_power_mode_enter_deep_sleep();
    }
}

void app_main(void)
{
    xTaskCreate(low_power_mode_task, "deep_sleep_task", 2048, NULL, 5, NULL);
}