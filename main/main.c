#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "bme280.h"
#include "low_power_mode.h"

#define TAG_BME280 "BME280"
#define TAG "MAIN"

void main_task()
{
    uint32_t com_rslt;

    com_rslt = bme280_init_driver(CONFIG_BME280_I2C_ADDRESS);
    com_rslt += bme280_set_oversamp(3, 3, 3);
    com_rslt += bme280_set_settings(0, 4, 2);

    ESP_LOGI(TAG_BME280, "BME280 init result: %ld", com_rslt);
    if (com_rslt == 0)
    {
        ESP_LOGI(TAG_BME280, "BME280 init success");
        for (int i = 1;; i++)
        {
            ESP_LOGI(TAG_BME280, "Temperature: %.2f C, Pressure: %.2f hPa, Humidity: %.2f %%", bme280_read_temperature(), bme280_read_pressure() / 100, bme280_read_humidity());
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            if (i % 10 == 0)
            {
                low_power_mode_set_sleep_time(10);
                low_power_mode_enter_deep_sleep();
            }
        }
    }
    else
    {
        ESP_LOGI(TAG_BME280, "BME280 init failed. code: %ld", com_rslt);
        esp_restart();
    }
}

// Main application
void app_main(void)
{
    TaskHandle_t main_task_handle = NULL;
    xTaskCreate(&main_task, "main_task", 4096, NULL, 4, &main_task_handle);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Free heap: %" PRIu32 "", esp_get_free_heap_size());
        if (eTaskGetState(main_task_handle) == eDeleted)
        {
            ESP_LOGI(TAG, "Task deleted");
        }
    }
}