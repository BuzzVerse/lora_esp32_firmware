#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "bme280.h"

#define TAG_BME280 "BME280"

void bme280_task()
{
    uint32_t com_rslt;

    com_rslt = bme280_init_driver(CONFIG_BME280_I2C_ADDRESS);
    com_rslt += bme280_set_oversamp();
    com_rslt += bme280_set_settings();

    ESP_LOGI(TAG_BME280, "BME280 init result: %ld", com_rslt);
    if (com_rslt == 0)
    {
        ESP_LOGI(TAG_BME280, "BME280 init success");
        for (;;)
        {
            ESP_LOGI(TAG_BME280, "Temperature: %.2f C", bme280_read_temperature());
            ESP_LOGI(TAG_BME280, "Pressure: %.2f hPa", bme280_read_pressure() / 100);
            ESP_LOGI(TAG_BME280, "Humidity: %.2f %%", bme280_read_humidity());
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    else
    {
        ESP_LOGI(TAG_BME280, "BME280 init failed. code: %ld", com_rslt);
        vTaskDelete(NULL);
    }
}

// Main application
void app_main(void)
{
    xTaskCreate(&bme280_task, "bme280_task", 4096, NULL, 4, NULL);
}