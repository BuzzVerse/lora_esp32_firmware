#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lora.h"
#include "bme280.h"
#include "low_power_mode.h"

#define TAG_BME280 "BME280"
#define TAG_MAIN "MAIN"

#ifndef CONFIG_RECEIVE_MODE

// Receiver task
void main_task()
{
}

#else

// Transmitter task
void main_task()
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        lora_send_packet((uint8_t *)"Hello", 5);
        printf("packet sent...\n");
    }
}

#endif

// Main application
void app_main(void)
{
    TaskHandle_t main_task_handle = NULL;
    int x_task_returned = xTaskCreate(&main_task, "main_task", 2048, NULL, 4, &main_task_handle);

    lora_init();
    lora_set_frequency(915e6);
    lora_enable_crc();

    ESP_LOGI(TAG_BME280, "Hello World0");

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