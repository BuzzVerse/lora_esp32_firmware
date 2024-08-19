// main.c

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bme280.h"
#include "lora.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "protocol_examples_common.h"
#include "low_power_mode.h"
#include "i2c.h"
#include "bq27441.h"
#include "bmm150.h"

#define TAG_MAIN "Main"

void app_main()
{
    struct bmm150_dev dev;
    struct bmm150_settings settings;
    struct bmm150_mag_data data;

    i2c_init();

    // Initialize the BMM150 driver
    if (bmm150_init_driver(0x10) == ESP_OK)
    {
        // Set up the configuration
        if (set_config(&settings) == ESP_OK)
        {
                ESP_LOGI(TAG_MAIN, "Configuration set successfully.");
        }
        else
        {
            ESP_LOGE(TAG_MAIN, "Failed to set configuration.");
        }
    }
    else
    {
        ESP_LOGE(TAG_MAIN, "Failed to initialize BMM150 driver.");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    for (;;)
    {
        bmm150_read_mag_data_driver(&data);
        ESP_LOGI(TAG_MAIN, "X: %d, Y: %d, Z: %d", data.x, data.y, data.z);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}