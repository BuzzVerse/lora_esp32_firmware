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

#define TAG "Main"

#if CONFIG_LORA_TRANSMITTER || CONFIG_TESTER
static void initialize_sensors(void);
#endif

#if CONFIG_LORA_TRANSMITTER
static void task_tx(void *pvParameters);
#endif

#if CONFIG_LORA_RECEIVER
static void task_rx(void *pvParameters);
#endif

#if CONFIG_TESTER
static void task_tester(void *pvParameters);
#endif

#if CONFIG_LORA_TRANSMITTER || CONFIG_TESTER
static void initialize_sensors(void)
{
    esp_err_t bme_rc = ESP_OK;
    bme_rc += bme280_init_driver(CONFIG_BME280_I2C_ADDRESS);
    bme_rc += bme280_set_oversamp(BME280_OVERSAMP_16X, BME280_OVERSAMP_16X, BME280_OVERSAMP_16X);
    bme_rc += bme280_set_settings(STANDBY_10MS, BME280_FILTER_COEFF_16, BME280_NORMAL_MODE);

    if (ESP_OK != bme_rc)
    {
        ESP_LOGE("BME", "BME280 settings failed");
        while (1)
        {
            vTaskDelay(1);
        }
    }
}
#endif

#if CONFIG_TESTER || CONFIG_LORA_RECEIVER
static const char *MQTT_TAG = "MQTT";
static esp_mqtt_client_handle_t mqtt_client;

static void initialize_mqtt(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.hostname = "158.180.60.2",
        .broker.address.port = 1883,
        .broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
        .credentials.username = "admin",
        .credentials.authentication.password = "",
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
}
#endif

void app_main(void)
{
    ESP_LOGI("MAIN", "Initializing LoRa");
    if (LORA_OK != lora_init())
    {
        ESP_LOGE("MAIN", "LoRa initialization failed");
        return;
    }

#if CONFIG_LORA_TRANSMITTER
    initialize_sensors();
    xTaskCreate(&task_tx, "TX", 1024 * 3, NULL, 5, NULL);
#endif

#if CONFIG_LORA_RECEIVER
    xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
#endif

#if CONFIG_TESTER
    initialize_mqtt();
    initialize_sensors();
    xTaskCreate(&task_tester, "Tester", 1024 * 3, NULL, 5, NULL);
#endif
}

#if CONFIG_LORA_TRANSMITTER
void task_tx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start TX");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    lora_packet_t packet = {0};

    while (1)
    {
        double temp_raw, press_raw, hum_raw;
        bme280_read_temperature(&temp_raw);
        bme280_read_pressure(&press_raw);
        bme280_read_humidity(&hum_raw);

        // Fill the packet with the meta data
        packet.version = (CONFIG_PACKET_VERSION << 4) | 0; // Reserved 4 bits set to 0
        packet.id = (CONFIG_CLASS_ID << 4) | CONFIG_DEVICE_ID;
        packet.msgID = 1;                           // Example message ID
        packet.msgCount = 1;                        // Example message count (optional, set as needed)
        packet.dataType = CONFIG_DATA_TYPE;                // Example data type

        // Convert raw sensor readings
        packet.data[0] = (int8_t)(temp_raw * 2);             // Scale temperature for higher precision and fit into int8_t
        packet.data[1] = (int8_t)((press_raw / 100) - 1000); // Convert Pa to hPa, subtract 1000
        packet.data[2] = (uint8_t)hum_raw;                   // Fit humidity into uint8_t

        // Log the packet data before sending
        ESP_LOGI(pcTaskGetName(NULL), "Temperature: %.2f C (scaled to %d)", temp_raw, packet.data[0]);
        ESP_LOGI(pcTaskGetName(NULL), "Pressure: %.2f hPa (stored as %d)", press_raw / 100, packet.data[1]);
        ESP_LOGI(pcTaskGetName(NULL), "Humidity: %.2f %% (stored as %u)", hum_raw, packet.data[2]);

        lora_send_confirmation(&packet);
        ESP_LOGI(TAG, "Packet sent");

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

#endif

#if CONFIG_LORA_RECEIVER
void task_rx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start RX");
    lora_packet_t packet = {0};

    while (1)
    {
        lora_receive_confirmation(&packet);

        // Unpack and log the received data
        float received_temp = ((float)((int8_t)packet.data[0]) / 2.0);
        float received_press = (float)(1000 + (int8_t)packet.data[1]);
        float received_hum = (float)packet.data[2];

        // Log the decoded values
        ESP_LOGI(pcTaskGetName(NULL), "Received Temp: %.2f C", received_temp);
        ESP_LOGI(pcTaskGetName(NULL), "Received Pressure: %.2f hPa", received_press);
        ESP_LOGI(pcTaskGetName(NULL), "Received Humidity: %.2f %%", received_hum);

        printf("\n");

        // Log the packet details
        ESP_LOGI(pcTaskGetName(NULL), "Class: %d", packet.id >> 4);
        ESP_LOGI(pcTaskGetName(NULL), "Device ID: %d", packet.id & 0x0F);
        ESP_LOGI(pcTaskGetName(NULL), "Version: %d", packet.version >> 4);
        ESP_LOGI(pcTaskGetName(NULL), "Message ID: %d", packet.msgID);
        ESP_LOGI(pcTaskGetName(NULL), "Message Count: %d", packet.msgCount);
        ESP_LOGI(pcTaskGetName(NULL), "Data Type: %d", packet.dataType);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
#endif

#if CONFIG_TESTER
void task_tester(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start Tester TX");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    while (1)
    {
        uint8_t tx_buf[24];
        bme280_read_temperature(&data.temperature.value);
        bme280_read_pressure(&data.pressure.value);
        bme280_read_humidity(&data.humidity.value);

        ESP_LOGI(pcTaskGetName(NULL), "Temperature: %.2f C", data.temperature.value);
        ESP_LOGI(pcTaskGetName(NULL), "Pressure: %.2f hPa", data.pressure.value / 100);
        ESP_LOGI(pcTaskGetName(NULL), "Humidity: %.2f %%", data.humidity.value);

        memcpy(tx_buf, data.temperature.serialized, 8);
        memcpy(tx_buf + 8, data.pressure.serialized, 8);
        memcpy(tx_buf + 16, data.humidity.serialized, 8);

        char msg[100];
        sprintf(msg, "{\"temperature\":%.2f, \"pressure\":%.2f, \"humidity\":%.2f}",
                data.temperature.value, data.pressure.value / 100, data.humidity.value);

        ESP_LOGI(pcTaskGetName(NULL), "Publishing message: %s", msg);

        int msg_id = esp_mqtt_client_publish(mqtt_client, "tele/lora/SENSOR_SPANISH", msg, 0, 1, 0);
        if (msg_id < 0)
        {
            ESP_LOGE(pcTaskGetName(NULL), "Failed to publish message");
        }
        else
        {
            ESP_LOGI(pcTaskGetName(NULL), "Published message with msg_id: %d", msg_id);
        }
        low_power_mode_set_sleep_time(5 * 60);
        low_power_mode_enter_deep_sleep();
    }
}
#endif
