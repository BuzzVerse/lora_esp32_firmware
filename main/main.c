#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "bme280.h"
#include "driver/lora_driver.h"
#include "low_power_mode.h"
#include "protocol_examples_common.h"

#define TESTER 0

#if CONFIG_LORA_TRANSMITTER
static void task_tx(void *pvParameters);
static void initialize_sensors(void);
#endif
#if CONFIG_LORA_RECEIVER
static void task_rx(void *pvParameters);
#endif
#if TESTER
static void task_tester(void *pvParameters);
static void initialize_sensors(void);
#endif

#if CONFIG_LORA_TRANSMITTER || TESTER
#define DATA_SIZE 60

// Define the bitfields for the first part of the packet
typedef struct
{
    uint8_t version : 3;    // 3 bits for version
    uint8_t packetType : 1; // 1 bit for packet type
    uint8_t msgID : 4;      // 4 bits for message ID
} PacketHeader;

// Define the entire packet structure
typedef struct
{
    PacketHeader header;     // 1 byte for header (3 bits version, 1 bit packet type, 4 bits msg ID)
    uint16_t class;          // 2 bytes for class
    uint16_t deviceID;       // 2 bytes for device ID
    uint8_t packetCategory;  // 1 byte for packet category
    uint8_t data[DATA_SIZE]; // 60 bytes for data
} LoRaPacket;

typedef struct
{
    int8_t temperature; // Temperature scaled to int8_t
    int8_t pressure;    // Pressure directly in uint16_t
    uint8_t humidity;   // Humidity directly in uint8_t
} bme280_data_t;

static bme280_data_t data;
static void initialize_sensors(void);
#endif

#if TESTER || CONFIG_LORA_RECEIVER
static const char *MQTT_TAG = "MQTT";
static esp_mqtt_client_handle_t mqtt_client;
static void initialize_mqtt(void);
#endif

// Main application entry
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing LoRa");
    if (ESP_OK != lora_init())
    {
        ESP_LOGE(TAG, "LoRa initialization failed");
        return;
    }

    lora_set_frequency(433e6);
    lora_enable_crc();
    lora_set_coding_rate(8);
    lora_set_bandwidth(2);
    lora_set_spreading_factor(12);

    lora_dump_registers();

#if CONFIG_LORA_TRANSMITTER
    initialize_sensors();
    xTaskCreate(&task_tx, "TX", 1024 * 3, NULL, 5, NULL);
#endif
#if CONFIG_LORA_RECEIVER
    // initialize_mqtt();
    xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
#endif
#if TESTER
    initialize_mqtt();
    initialize_sensors();
    xTaskCreate(&task_tester, "Tester", 1024 * 3, NULL, 5, NULL);
#endif
}

#if TESTER || CONFIG_LORA_TRANSMITTER
static void initialize_sensors(void)
{
    esp_err_t bme_rc = ESP_OK;
    bme_rc += bme280_init_driver(CONFIG_BME280_I2C_ADDRESS);

    bme_rc += bme280_set_oversamp(
        BME280_OVERSAMP_16X,
        BME280_OVERSAMP_16X,
        BME280_OVERSAMP_16X);

    bme_rc += bme280_set_settings(
        STANDBY_10MS,
        BME280_FILTER_COEFF_16,
        BME280_NORMAL_MODE);

    if (ESP_OK != bme_rc)
    {
        ESP_LOGE("BME", "BME280 settings failed");
        while (1)
        {
            // @TODO Add BME280 error handling
            vTaskDelay(1);
        }
    }
}
#endif

#if TESTER || CONFIG_LORA_RECEIVER
static void initialize_mqtt(void)
{
    // setup wifi and related settings for mqtt
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.hostname = "158.180.60.2",
        .broker.address.port = 1883,
        .broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
        // YOU NEED TO CHANGE THESE
        // @TODO: Implement menuconfig variables for these
        .credentials.username = "admin",
        .credentials.authentication.password = "",
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
}
#endif

#if CONFIG_LORA_TRANSMITTER
void task_tx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start TX");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    uint8_t tx_buf[3] = {0};

    while (1)
    {
        double temp_raw, press_raw, hum_raw;
        bme280_read_temperature(&temp_raw);
        bme280_read_pressure(&press_raw);
        bme280_read_humidity(&hum_raw);

        ESP_LOGI(TAG, "Raw Temp: %.2f, Raw Press: %.2f, Raw Hum: %.2f", temp_raw, press_raw, hum_raw);

        // // Convert raw sensor readings
        // data.temperature = (int8_t)(temp_raw * 2);          // Scale temperature for higher precision and fit into int8_t
        // data.pressure = (int8_t)((press_raw / 100) - 1000); // Convert Pa to hPa, subtract 1000
        // data.humidity = (uint8_t)(hum_raw);                 // Fit humidity into uint8_t

        // // Packing the data into tx_buf
        // tx_buf[0] = (uint8_t)data.temperature;
        // tx_buf[1] = (uint8_t)data.pressure; // Store the pressure difference directly in one byte
        // tx_buf[2] = data.humidity;

        LoRaPacket packet;
        uint8_t buffer[sizeof(LoRaPacket)] = {0};

        // Fill the packet with some example data
        packet.header.version = 3;
        packet.header.packetType = 1;
        packet.header.msgID = 5;
        packet.class = 256;
        packet.deviceID = 512;
        packet.packetCategory = 2;
        memset(packet.data, 'A', DATA_SIZE);
    }
}
#endif

#if CONFIG_LORA_RECEIVER
void task_rx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start RX");
    while (1)
    {
        lora_receive(); // Put into receive mode
        bool hasReceived = false;
        lora_received(&hasReceived);

        if (hasReceived)
        {
            uint8_t rx_buf[3] = {0};
            uint8_t rxLen = 0;
            uint8_t rssi = 0;
            uint8_t snr = 0;

            lora_receive_packet(rx_buf, &rxLen, sizeof(rx_buf));

            lora_packet_rssi(&rssi);
            lora_packet_snr(&snr);

            ESP_LOGI(TAG, "RSSI: %d dBm, SNR: %d dB", rssi, snr);
            ESP_LOGI(TAG, "RX Buffer: 0x%02X 0x%02X 0x%02X", rx_buf[0], rx_buf[1], rx_buf[2]);

            float received_temp = ((float)((int8_t)rx_buf[0]) / 2.0);
            int8_t packed_pressure = (int8_t)rx_buf[1];      // Force correct signed interpretation
            int16_t received_press = 1000 + packed_pressure; // Correctly adjust by adding 1000

            float received_hum = (float)rx_buf[2];

            // Log the decoded values
            ESP_LOGI(pcTaskGetName(NULL), "Received Temp: %.2f C, Pressure: %d hPa, Humidity: %.2f %%", received_temp, received_press, received_hum);

            // // Format the message to be sent over MQTT
            // char msg[100];
            // sprintf(msg, "{\"temperature\":%.2f, \"pressure\":%.2f, \"humidity\":%.2f}",
            //         data.temperature.value, data.pressure.value / 100, data.humidity.value);

            // // Publish the formatted message over MQTT
            // int msg_id = esp_mqtt_client_publish(mqtt_client, "tele/lora/SENSOR_SPANISH", msg, 0, 1, 0);
            // if (msg_id < 0)
            //     ESP_LOGE(pcTaskGetName(NULL), "Failed to publish message");
            // else
            //     ESP_LOGI(pcTaskGetName(NULL), "Published message with msg_id: %d", msg_id);
        }
        vTaskDelay(2);
    }
}
#endif

#if TESTER
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

        // Format the message to be sent over MQTT
        char msg[100];
        sprintf(msg, "{\"temperature\":%.2f, \"pressure\":%.2f, \"humidity\":%.2f}",
                data.temperature.value, data.pressure.value / 100, data.humidity.value); // Adjusting pressure to hPa

        ESP_LOGI(pcTaskGetName(NULL), "Publishing message: %s", msg);

        // Publish the formatted message over MQTT
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