#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "bme280.h"
#include "low_power_mode.h"
#include "driver/gpio.h"

#define TAG_BME280 "BME280"
#define TAG_MAIN "MAIN"

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lora_driver.h"

#define CONFIG_SENDER 1
#define CONFIG_433MHZ 1

#if CONFIG_SENDER
void task_tx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    uint8_t buf[256] = {0}; // Maximum Payload size of SX1276/77/78/79 is 255

    while (1)
    {
        TickType_t nowTick = xTaskGetTickCount();
        int send_len = sprintf((char *)buf, "Puszczam szczura %" PRIu32, nowTick);
        lora_send_packet(buf, send_len);
        ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", send_len);
        int lost = lora_packet_lost();
        if (lost != 0)
        {
            ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    } // end while
}
#endif // CONFIG_SENDER

#if CONFIG_RECEIVER
void task_rx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    uint8_t buf[256]; // Maximum Payload size of SX1276/77/78/79 is 255

    while (1)
    {
        lora_receive(); // put into receive mode
        bool hasReceived;
        lora_received(&hasReceived);
        
        if (hasReceived)
        {
            int rxLen;
            lora_receive_packet(buf, &rxLen, sizeof(buf));
            ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s]", rxLen, rxLen, buf);
        }
        vTaskDelay(1); // Avoid WatchDog alerts
    }                  // end while
}
#endif // CONFIG_RECEIVER

// Main application
void app_main(void)
{
    if (lora_init() != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
		while(1) {
			vTaskDelay(1);
		}
	}

#if CONFIG_169MHZ
	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 169MHz");
	lora_set_frequency(169e6); // 169MHz
#elif CONFIG_433MHZ
	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 433MHz");
	lora_set_frequency(433e6); // 433MHz
#elif CONFIG_470MHZ
	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 470MHz");
	lora_set_frequency(470e6); // 470MHz
#elif CONFIG_866MHZ
	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 866MHz");
	lora_set_frequency(866e6); // 866MHz
#elif CONFIG_915MHZ
	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
	lora_set_frequency(915e6); // 915MHz
#elif CONFIG_OTHER
	ESP_LOGI(pcTaskGetName(NULL), "Frequency is %dMHz", CONFIG_OTHER_FREQUENCY);
	long frequency = CONFIG_OTHER_FREQUENCY * 1000000;
	lora_set_frequency(frequency);
#endif

	lora_enable_crc();

	int cr = 8;
	int bw = 0;
	int sf = 12;
#if CONFIF_ADVANCED
	cr = CONFIG_CODING_RATE
	bw = CONFIG_BANDWIDTH;
	sf = CONFIG_SF_RATE;
#endif

	lora_set_coding_rate(cr);
	//lora_set_coding_rate(CONFIG_CODING_RATE);
	//cr = lora_get_coding_rate();
	ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

	lora_set_bandwidth(bw);
	//lora_set_bandwidth(CONFIG_BANDWIDTH);
	//int bw = lora_get_bandwidth();
	ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

	lora_set_spreading_factor(sf);
	//lora_set_spreading_factor(CONFIG_SF_RATE);
	//int sf = lora_get_spreading_factor();
	ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

#if CONFIG_SENDER
	xTaskCreate(&task_tx, "TX", 1024*3, NULL, 5, NULL);
#endif
#if CONFIG_RECEIVER
	xTaskCreate(&task_rx, "RX", 1024*3, NULL, 5, NULL);
#endif


    // TaskHandle_t main_task_handle = NULL;
    // int x_task_returned = xTaskCreate(&main_task, "main_task", 2048, NULL, 4, &main_task_handle);

    // ESP_LOGI(TAG_BME280, "Hello World0");

    // if (x_task_returned != pdPASS)
    // {
    //     ESP_LOGE(TAG_BME280, "Error with creating the task. Returned with error code: %d", x_task_returned);
    //     vTaskDelete(main_task_handle);
    //     esp_restart();
    // }

    // for (;;)
    // {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     ESP_LOGI(TAG_MAIN, "Free heap: %" PRIu32 "", esp_get_free_heap_size());
    // }
}