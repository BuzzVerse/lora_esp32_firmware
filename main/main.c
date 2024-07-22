#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bq27441.h"
#include "esp_log.h"
#include "lora.h"

static const char *TAG_MAIN = "Main";

void app_main(void)
{
    esp_err_t err = bq27441_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MAIN, "Failed to initialize BQ27441");
        return;
    }

    if (LORA_OK != lora_init())
    {
        ESP_LOGE("MAIN", "LoRa initialization failed");
        esp_restart();
    }

    for (;;)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        packet_t packet = {0};

        // Fill the packet with the meta data
        packet.version = (CONFIG_PACKET_VERSION << 4) | 0; // Reserved 4 bits set to 0
        packet.id = (CONFIG_CLASS_ID << 4) | CONFIG_DEVICE_ID;
        packet.msgID = 1;                   // Example message ID
        packet.msgCount = 1;                // Example message count (optional, set as needed)
        packet.dataType = 5; // Example data type 

        uint16_t capacity;
        err = bq27441_read_design_capacity(&capacity);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG_MAIN, "Design Capacity: %d mAh", capacity);
        }
        else
        {
            ESP_LOGE(TAG_MAIN, "Failed to read design capacity");
        }

        uint8_t soc;
        err = bq27441_read_soc(&soc);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG_MAIN, "State of Charge: %d %%", soc);
        }
        else
        {
            ESP_LOGE(TAG_MAIN, "Failed to read state of charge");
        }

        uint16_t voltage;
        err = bq27441_read_voltage(&voltage);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG_MAIN, "Voltage: %d mV", voltage);
        }
        else
        {
            ESP_LOGE(TAG_MAIN, "Failed to read voltage");
        }

        packet.data[0] = soc;

        lora_status_t send_status = lora_send(&packet);
    }
}
