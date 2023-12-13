#include <stdio.h>
#include "low_power_mode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @file main.c
 * 
 * @brief This file contains the main function.
 * 
 * This file contains the main function. The main function creates a task that enters into the low power mode after 5 seconds.
 */

void app_main(void)
{

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    xTaskCreate(&low_power_mode_enter_deep_sleep, "deep_sleep_task", 2048, NULL, 5, NULL);
}