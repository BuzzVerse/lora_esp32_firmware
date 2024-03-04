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
#include "lora_driver.h"
#include "low_power_mode.h"
#include "protocol_examples_common.h"

static uint8_t rx_buf[24];
static const char *MQTT_TAG = "MQTT";
static esp_mqtt_client_handle_t mqtt_client;

typedef union
{
    double value;
    uint8_t serialized[8];

} serialized_data_t;

typedef struct
{
    serialized_data_t temperature;
    serialized_data_t humidity;
    serialized_data_t pressure;

} bme280_data_t;

static bme280_data_t data;

// Function Declarations
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
#if CONFIG_LORA_TRANSMITTER
static void task_tx(void *pvParameters);
#endif
#if CONFIG_LORA_RECEIVER
static void task_rx(void *pvParameters);
#endif
static void initialize_sensors(void);
static void initialize_mqtt(void);
static void log_error_if_nonzero(const char *message, int error_code);

// Main application entry
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing LoRa");
    if (ESP_OK != lora_init())
    {
        ESP_LOGE(TAG, "LoRa initialization failed");
        return;
    }

    lora_set_frequency(433e6); // 433MHz
    lora_enable_crc();
    lora_set_coding_rate(1);
    lora_set_bandwidth(7);
    lora_set_spreading_factor(7);
    lora_dump_registers();

#if CONFIG_LORA_TRANSMITTER
    initialize_sensors();
    xTaskCreate(&task_tx, "TX", 1024 * 3, NULL, 5, NULL);
#endif
#if CONFIG_LORA_RECEIVER
    initialize_mqtt();
    xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
#endif
}

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
            vTaskDelay(1);
        }
    }
}

static void initialize_mqtt(void)
{
    // setup wifi and related settings for mqtt
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.hostname = "",
        .broker.address.port = 1883,
        .broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
        // YOU NEED TO CHANGE THESE
        // TODO: Implement menuconfig variables for these
        .credentials.username = "",
        .credentials.authentication.password = "",
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    printf("Event topic: %s", event->topic);
    printf("Event data: %s", event->data);

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, event->topic, event->data, 0, 1, 0);
        ESP_LOGI(MQTT_TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(MQTT_TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DATA");
        printf("\nTOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    default:
        ESP_LOGI(MQTT_TAG, "Other event id:%d", event->event_id);
        break;
    }
}

#if CONFIG_LORA_TRANSMITTER
void task_tx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start TX");

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

        lora_send_packet(tx_buf, 24);

        int lost = lora_packet_lost();

        if (lost != 0)
        {
            ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
        }
        vTaskDelay(pdMS_TO_TICKS(15000));
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
            uint8_t rxLen = 0;
            lora_receive_packet(rx_buf, &rxLen, sizeof(rx_buf));

            // Deserialize the byte array into a struct
            memcpy(data.temperature.serialized, rx_buf, 8);
            memcpy(data.pressure.serialized, rx_buf + 8, 8);
            memcpy(data.humidity.serialized, rx_buf + 16, 8);

            ESP_LOGI(pcTaskGetName(NULL), "Received Temp: %f, Pres: %f, Hum: %f",
                     data.temperature.value, data.pressure.value, data.humidity.value);

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
        }
        vTaskDelay(2);
    }
}
#endif

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(MQTT_TAG, "Last error %s: 0x%x", message, error_code);
    }
}
