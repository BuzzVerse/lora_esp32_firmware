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
#include "crypto.h"

#define TAG "Main"

#if CONFIG_LORA_TRANSMITTER
static void task_tx(void *pvParameters);

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

#if CONFIG_LORA_RECEIVER
static void task_rx(void *pvParameters);
#endif

#if CONFIG_LORA_RECEIVER && CONFIG_ENABLE_MQTT
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
        .credentials.authentication.password = "***REMOVED***",
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
}
#endif

void app_main(void)
{
    uint8_t plaintext[] = "This is a test.";
    size_t plaintext_size = sizeof(plaintext);
    uint8_t ciphertext[plaintext_size];
    uint8_t decryptedtext[plaintext_size];
    uint8_t iv[16] = {0}; // Initial nonce counter

    // Encrypt
    crypto_status_t status = crypto_crypt(plaintext, plaintext_size, ciphertext, iv, CRYPTO_OPERATION_ENCRYPT);

    // Decrypt
    status = crypto_crypt(ciphertext, plaintext_size, decryptedtext, iv, CRYPTO_OPERATION_DECRYPT);

    // Check if decrypted text matches the original plaintext
    bool match = true;
    for (size_t i = 0; i < plaintext_size; i++)
    {
        if (plaintext[i] != decryptedtext[i])
        {
            match = false;
            break;
        }
    }

    if (match)
        ESP_LOGI(TAG, "Decryption successful, texts match!");
    else
        ESP_LOGE(TAG, "Decryption failed, texts do not match!");

    //     ESP_LOGI("MAIN", "Initializing LoRa");
    //     if (LORA_OK != lora_init())
    //     {
    //         ESP_LOGE("MAIN", "LoRa initialization failed");
    //         // Maybe add a retry mechanism here? Either for the lora only, and retry the init function, or for the whole device, and reboot it.
    //     }

    // #if CONFIG_LORA_TRANSMITTER
    //     initialize_sensors();
    //     xTaskCreate(&task_tx, "TX", 1024 * 3, NULL, 5, NULL);
    // #endif

    // #if CONFIG_LORA_RECEIVER && CONFIG_ENABLE_MQTT
    //     initialize_mqtt();
    // #endif

    // #if CONFIG_LORA_RECEIVER
    //     xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
    // #endif
}

#if CONFIG_LORA_TRANSMITTER
void task_tx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start TX");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    packet_t packet = {0};

    while (1)
    {
        double temp_raw, press_raw, hum_raw;
        bme280_read_temperature(&temp_raw);
        bme280_read_pressure(&press_raw);
        bme280_read_humidity(&hum_raw);

        // Fill the packet with the meta data
        packet.version = (CONFIG_PACKET_VERSION << 4) | 0; // Reserved 4 bits set to 0
        packet.id = (CONFIG_CLASS_ID << 4) | CONFIG_DEVICE_ID;
        packet.msgID = 1;                   // Example message ID
        packet.msgCount = 1;                // Example message count (optional, set as needed)
        packet.dataType = CONFIG_DATA_TYPE; // Example data type

        if (packet.dataType == BME280)
        {
            // Convert raw sensor readings
            packet.data[0] = (int8_t)(temp_raw * 2);             // Scale temperature for higher precision and fit into int8_t
            packet.data[1] = (int8_t)((press_raw / 100) - 1000); // Convert Pa to hPa, subtract 1000
            packet.data[2] = (uint8_t)hum_raw;                   // Fit humidity into uint8_t

            // Log the packet data before sending
            ESP_LOGI(pcTaskGetName(NULL), "Temperature: %.2f C (scaled to %d)", temp_raw, packet.data[0]);
            ESP_LOGI(pcTaskGetName(NULL), "Pressure: %.2f hPa (stored as %d)", press_raw / 100, packet.data[1]);
            ESP_LOGI(pcTaskGetName(NULL), "Humidity: %.2f %% (stored as %u)", hum_raw, packet.data[2]);
        }
        else if (packet.dataType == SMS)
        {
            // pack a String "BuzzVerse" into data array
            char data[] = "BuzzVerse";
            for (int i = 0; i < sizeof(data); i++)
            {
                packet.data[i] = data[i];
            }

            // Log the packet data before sending
            ESP_LOGI(pcTaskGetName(NULL), "Data: %s", packet.data);
        } // Add more data types here

        lora_status_t send_status = lora_send(&packet);

        if (LORA_OK != send_status)
        {
            ESP_LOGE(TAG, "Packet send failed");
        }
        else
        {
            ESP_LOGI(TAG, "Packet sent successfully");
        }

        // low_power_mode_set_sleep_time(15 * 60); // 15 minutes
        // low_power_mode_enter_deep_sleep();
    }
}

#endif

#if CONFIG_LORA_RECEIVER
void task_rx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start RX");
    packet_t packet = {0};

    while (1)
    {
        lora_status_t status;

        char msg[100];

        status = lora_receive(&packet);

        // Log the packet details
        ESP_LOGI(pcTaskGetName(NULL), "Class: %d", packet.id >> 4);
        ESP_LOGI(pcTaskGetName(NULL), "Device ID: %d", packet.id & 0x0F);
        ESP_LOGI(pcTaskGetName(NULL), "Version: %d", packet.version >> 4);
        ESP_LOGI(pcTaskGetName(NULL), "Message ID: %d", packet.msgID);
        ESP_LOGI(pcTaskGetName(NULL), "Message Count: %d", packet.msgCount);
        ESP_LOGI(pcTaskGetName(NULL), "Data Type: %d", packet.dataType);

        if (packet.dataType == BME280)
        {
            // Unpack and log the received data
            float received_temp = ((float)((int8_t)packet.data[0]) / 2.0);
            float received_press = (float)(1000 + (int8_t)packet.data[1]);
            float received_hum = (float)packet.data[2];

            // Log the decoded values
            ESP_LOGI(pcTaskGetName(NULL), "Received Temp: %.2f C", received_temp);
            ESP_LOGI(pcTaskGetName(NULL), "Received Pressure: %.2f hPa", received_press);
            ESP_LOGI(pcTaskGetName(NULL), "Received Humidity: %.2f %%", received_hum);

            if (status == LORA_OK)
            {
                sprintf(msg, "{\"temperature\":%.2f, \"pressure\":%.2f, \"humidity\":%.2f}",
                        received_temp, received_press, received_hum);
            }
            else
            {
                ESP_LOGE(LORA_TAG, "Message CRC error!");
                sprintf(msg, "{\"temperature\":-1, \"pressure\":-1, \"humidity\":-1}");
            }
        }
        else if (packet.dataType == SMS)
        {
            // Unpack and log the received data
            char received_data[10];
            for (int i = 0; i < sizeof(received_data); i++)
            {
                received_data[i] = packet.data[i];
            }

            // Log the decoded values
            ESP_LOGI(pcTaskGetName(NULL), "Received Data: %s", received_data);

            if (status == LORA_OK)
            {
                sprintf(msg, "{\"data\":\"%s\"}", received_data);
            }
            else
            {
                ESP_LOGE(LORA_TAG, "Message CRC error!");
                sprintf(msg, "{\"data\":\"\"}");
            }
        }

#if CONFIG_ENABLE_MQTT

        int msg_id = esp_mqtt_client_publish(mqtt_client, "tele/lora/SENSOR_SPANISH", msg, 0, 1, 0);
        if (msg_id < 0)
        {
            ESP_LOGE(MQTT_TAG, "Failed to publish message");
        }
        else
        {
            ESP_LOGI(MQTT_TAG, "Published message with msg_id: %d", msg_id);
        }

#endif
    }
}
#endif
