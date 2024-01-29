#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "bme280.h"
#include "low_power_mode.h"

#define TAG_BME280 "BME280"
#define TAG_MAIN "MAIN"

void main_task()
{
    esp_err_t esp_rc;
    double temperature, pressure, humidity;

    esp_rc = bme280_init_driver(CONFIG_BME280_I2C_ADDRESS);
    esp_rc += bme280_set_oversamp(OVERSAMP_4X, OVERSAMP_4X, OVERSAMP_4X);
    esp_rc += bme280_set_settings(STANDBY_10MS, FILTER_16, MODE_FORCED);

    // wait 100ms for the sensor to reach a stable state
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG_BME280, "BME280 init result: %d", esp_rc);
    if (esp_rc == 0)
    {
        ESP_LOGI(TAG_BME280, "BME280 init success");
        for (int i = 1;; i++)
        {
            bme280_read_temperature(&temperature);
            bme280_read_pressure(&pressure);
            bme280_read_humidity(&humidity);

            ESP_LOGI(TAG_BME280, "Temperature: %.2f C, Pressure: %.2f hPa, Humidity: %.2f %%", temperature, pressure / 100, humidity);

            vTaskDelay(10000 / portTICK_PERIOD_MS);

            low_power_mode_set_sleep_time(10);
            low_power_mode_enter_deep_sleep();
        }
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 init failed. code: %d", esp_rc);
        esp_restart();
    }
}

// Main application
void app_main(void)
{
    TaskHandle_t main_task_handle = NULL;
    int x_task_returned = xTaskCreate(&main_task, "main_task", 2048, NULL, 4, &main_task_handle);

    if (x_task_returned != pdPASS)
    {
        ESP_LOGE(TAG_BME280, "Error with creating the task. Returned with error code: %d", x_task_returned);
        vTaskDelete(main_task_handle);
        esp_restart();
    }

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG_MAIN, "Free heap: %" PRIu32 "", esp_get_free_heap_size());
    }
}