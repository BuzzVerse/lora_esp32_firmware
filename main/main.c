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

#define MAX_VALUE 100
#define HISTOGRAM_WIDTH 50

// prosty plot z gpt
void draw_histogram(int x, int y, int z) {
    int i;

    printf("\033[H\033[J"); // Czyść ekran terminala

    printf("Histogram dla wartości XYZ:\n");
    
    printf("X: ");
    for (i = 0; i < (x * HISTOGRAM_WIDTH / MAX_VALUE); i++) {
        printf("#");
    }
    printf(" (%d)\n", x);

    printf("Y: ");
    for (i = 0; i < (y * HISTOGRAM_WIDTH / MAX_VALUE); i++) {
        printf("#");
    }
    printf(" (%d)\n", y);

    printf("Z: ");
    for (i = 0; i < (z * HISTOGRAM_WIDTH / MAX_VALUE); i++) {
        printf("#");
    }
    printf(" (%d)\n", z);
}

void app_main()
{
    static struct bmm150_dev dev;
    static struct bmm150_settings settings = {0};
    static struct bmm150_mag_data data;

    i2c_init();

    // Initialize the BMM150 driver
    if (bmm150_init_driver() == ESP_OK)
    {
      bmm150_debug();
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

    for (uint32_t i = 0;; i++)
    {
      if (0 == i % 100) {
    	  bmm150_read_mag_data_driver(&data);
    	  //ESP_LOGI(TAG_MAIN, "X: %d, Y: %d, Z: %d", data.x, data.y, data.z);
    	  draw_histogram(data.x, data.y, data.z);
      }
      vTaskDelay(1);
    }
}


