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

#if CONFIG_LORA_TRANSMITTER
static void task_tx(void *pvParameters);

static void initialize_sensors(void)
{
    esp_err_t bme_rc = ESP_OK;
 
    i2c_init();

    esp_err_t bm_rc = ESP_OK;
    bm_rc = bmm150_init_driver(BMM150_DEFAULT_I2C_ADDRESS);
    if(ESP_OK != bm_rc)
    {
        ESP_LOGE("BMM280", "Failed to initialize BMM150");
        return;
    }


    if (ESP_OK != bme280_init_driver(CONFIG_BME280_I2C_ADDRESS))
    {
        ESP_LOGE(TAG_MAIN, "BME280 initialization failed");
        // not sure if this should restart, but also not sure what else to do here?
        esp_restart();
    }

    esp_err_t bq_rc = bq27441_init();
    if (bq_rc != ESP_OK)
    {
        ESP_LOGE(TAG_MAIN, "Failed to initialize BQ27441");
        return;
    }


    uint16_t capacity;
        bq_rc = bq27441_read_design_capacity(&capacity);
        if (bq_rc == ESP_OK)
        {
            ESP_LOGI(TAG_MAIN, "Design Capacity: %d mAh", capacity);
        }
        else
        {
            ESP_LOGE(TAG_MAIN, "Failed to read design capacity");
        }

        uint8_t soc;
        bq_rc = bq27441_read_soc(&soc);
        if (bq_rc == ESP_OK)
        {
            ESP_LOGI(TAG_MAIN, "State of Charge: %d %%", soc);
        }
        else
        {
            ESP_LOGE(TAG_MAIN, "Failed to read state of charge");
        }

        uint16_t voltage;
        bq_rc = bq27441_read_voltage(&voltage);
        if (bq_rc == ESP_OK)
        {
            ESP_LOGI(TAG_MAIN, "Voltage: %d mV", voltage);
        }
        else
        {
            ESP_LOGE(TAG_MAIN, "Failed to read voltage");
        }


    bme_rc += bme280_set_oversamp(BME280_OVERSAMP_16X, BME280_OVERSAMP_16X, BME280_OVERSAMP_16X);
    bme_rc += bme280_set_settings(STANDBY_10MS, BME280_FILTER_COEFF_16, BME280_NORMAL_MODE);

    if (ESP_OK != bme_rc)
    {
        ESP_LOGE("BME", "BME280 settings failed");
        // not sure if this should restart, but also not sure what else to do here?
        esp_restart();
    }
}
#endif

#if CONFIG_LORA_RECEIVER
static void task_rx(void *pvParameters);
#define MSG_BUFFER_SIZE 128
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
        // not sure if this should restart, but also not sure what else to do here?
        esp_restart();
    }

#if CONFIG_LORA_TRANSMITTER
    initialize_sensors();
    xTaskCreate(&task_tx, "TX", 1024 * 3, NULL, 5, NULL);
#endif

#if CONFIG_LORA_RECEIVER && CONFIG_ENABLE_MQTT
    initialize_mqtt();
#endif

#if CONFIG_LORA_RECEIVER
    xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
#endif
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

        if (BME280 == packet.dataType)
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
        else if (SMS == packet.dataType)
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
            ESP_LOGE(TAG_MAIN, "Packet send failed");
        }
        else
        {
            ESP_LOGI(TAG_MAIN, "Packet sent successfully");
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

        char msg[MSG_BUFFER_SIZE];

        status = lora_receive(&packet);

        // Log the packet details
        ESP_LOGI(pcTaskGetName(NULL), "Class: %d", packet.id >> 4);
        ESP_LOGI(pcTaskGetName(NULL), "Device ID: %d", packet.id & 0x0F);
        ESP_LOGI(pcTaskGetName(NULL), "Version: %d", packet.version >> 4);
        ESP_LOGI(pcTaskGetName(NULL), "Message ID: %d", packet.msgID);
        ESP_LOGI(pcTaskGetName(NULL), "Message Count: %d", packet.msgCount);
        ESP_LOGI(pcTaskGetName(NULL), "Data Type: %d", packet.dataType);

        if (BME280 == packet.dataType)
        {
            // Unpack and log the received data
            float received_temp = ((float)((int8_t)packet.data[0]) / 2.0);
            float received_press = (float)(1000 + (int8_t)packet.data[1]);
            float received_hum = (float)packet.data[2];

            // Log the decoded values
            ESP_LOGI(pcTaskGetName(NULL), "Received Temp: %.2f C", received_temp);
            ESP_LOGI(pcTaskGetName(NULL), "Received Pressure: %.2f hPa", received_press);
            ESP_LOGI(pcTaskGetName(NULL), "Received Humidity: %.2f %%", received_hum);

            if (LORA_OK == status)
            {
                snprintf(msg, sizeof(msg), "{\"temperature\":%.2f, \"pressure\":%.2f, \"humidity\":%.2f}",
                         received_temp, received_press, received_hum);
            }
            else
            {
                ESP_LOGE(LORA_TAG, "Message CRC error!");
                snprintf(msg, sizeof(msg), "{\"temperature\":-1, \"pressure\":-1, \"humidity\":-1}");
            }
        }
        else if (SMS == packet.dataType)
        {
            // Unpack and log the received data
            char received_data[DATA_SIZE];
            for (int i = 0; i < sizeof(received_data); i++)
            {
                received_data[i] = packet.data[i];
            }

            // Log the decoded values
            ESP_LOGI(pcTaskGetName(NULL), "Received Data: %s", received_data);

            if (LORA_OK == status)
            {
                snprintf(msg, sizeof(msg), "{\"data\":\"%s\"}", received_data);
            }
            else
            {
                ESP_LOGE(LORA_TAG, "Message CRC error!");
                snprintf(msg, sizeof(msg), "{\"data\":\"\"}");
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