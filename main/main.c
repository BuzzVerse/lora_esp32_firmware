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

#define TAG "Main"

//#define CONFIG_LORA_TRANSMITTER 1
#define xBOOTUP_NO_BME280	0x01	// Set when BME280 was not initialized
#define xBOOTUP_NO_LORA		0x02	// Set when LoRa init fails

static uint8_t bootup_status = {0};

static void self_test(void)
{
	ESP_LOGI(TAG, "--SELF TEST BEGIN--");
	if (bootup_status & xBOOTUP_NO_BME280)
	{
		ESP_LOGI(TAG, "--NO BME280");
		ESP_LOGI(TAG, "--Initializing BME280 driver: %s", ESP_OK == bme280_init_driver(CONFIG_BME280_I2C_ADDRESS)?"OK":"FAILED");
	}
	if (bootup_status & xBOOTUP_NO_LORA)
	{
		ESP_LOGI(TAG, "--NO LORA");
		ESP_LOGI(TAG, "--Initializing LoRa driver: %s", LORA_OK == lora_init()?"OK":"FAILED");
	}
	ESP_LOGI(TAG, "--SELF TEST END--");
}

#if CONFIG_LORA_TRANSMITTER
static void task_tx(void *pvParameters);

static packet_t packet = {0};

static void initialize_sensors(void)
{
    esp_err_t bme_rc = ESP_OK;

    bme_rc = bme280_init_driver(CONFIG_BME280_I2C_ADDRESS);
    if (ESP_OK == bme280_init_driver(CONFIG_BME280_I2C_ADDRESS)) {

    	bme_rc += bme280_set_oversamp(BME280_OVERSAMP_16X, BME280_OVERSAMP_16X, BME280_OVERSAMP_16X);
    	bme_rc += bme280_set_settings(STANDBY_10MS, BME280_FILTER_COEFF_16, BME280_NORMAL_MODE);
    }

    if (ESP_OK != bme_rc)
    {
        ESP_LOGE(TAG, "BME280 initialization failed");
        bootup_status |= xBOOTUP_NO_BME280;
    }
}
#endif

#if CONFIG_LORA_RECEIVER
static void task_rx(void *pvParameters);
#define MSG_BUFFER_SIZE 128
#endif

#if CONFIG_LORA_RECEIVER && CONFIG_ENABLE_MQTT

const esp_mqtt_client_config_t default_mqtt_cfg =
{
		.broker.address.hostname = "192.168.0.1",
		.broker.address.port = 1234,
		.broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
		.credentials.username = "admin",
		.credentials.authentication.password = ""
};

static esp_mqtt_client_handle_t mqtt_client;

static void initialize_mqtt(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    esp_mqtt_client_config_t mqtt_cfg = default_mqtt_cfg;
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing LoRa");
    if (LORA_OK != lora_init())
    {
        ESP_LOGE(TAG, "LoRa initialization failed");
        // not sure if this should restart, but also not sure what else to do here?
        bootup_status = xBOOTUP_NO_LORA;
        self_test();
    }

#if CONFIG_LORA_TRANSMITTER
    initialize_sensors();
    xTaskCreate(&task_tx, "TX", 1024 * 4, NULL, 5, NULL);
#endif

#if CONFIG_LORA_RECEIVER && CONFIG_ENABLE_MQTT
    initialize_mqtt();
#endif

#if CONFIG_LORA_RECEIVER
    xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
#endif
}

#if CONFIG_LORA_TRANSMITTER
static double temp_raw, press_raw, hum_raw;

void task_tx(void *pvParameters)
{
	UBaseType_t uxHighWaterMark1, uxHighWaterMark2;

    ESP_LOGI(pcTaskGetName(NULL), "Start TX");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    uxHighWaterMark1 = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "TX task WaterMark1:0x%x", uxHighWaterMark1);

    while (1)
    {
        bme280_read_temperature(&temp_raw);
        bme280_read_pressure(&press_raw);
        bme280_read_humidity(&hum_raw);

        // Fill the packet with the meta data
        packet.version = (CONFIG_PACKET_VERSION << 4) | 0; // Reserved 4 bits set to 0
        packet.id = (CONFIG_CLASS_ID << 4) | CONFIG_DEVICE_ID;
        packet.msgID = 'B';	            // 'B' for Buzz;
        packet.msgCount = 'V';			// 'V' for Verse
        packet.dataType = CONFIG_DATA_TYPE;

        //packet.dataType = SMS;
        if (bootup_status & xBOOTUP_NO_BME280)
        {
        	packet.dataType = SMS;
			sprintf((char * restrict)packet.data, "BOOTUP:0x%x", bootup_status);
			// Log the packet data before sending
			ESP_LOGI(pcTaskGetName(NULL), "Data: %s", packet.data);
        }

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
        else
        {
			sprintf((char * restrict)packet.data, "Unsupported type");
			// Log the packet data before sending
			ESP_LOGI(pcTaskGetName(NULL), "Data: %s", packet.data);
        }
        lora_status_t send_status = lora_send(&packet);

        if (LORA_OK != send_status)
        {
            ESP_LOGE(TAG, "Packet send failed");
        }
        else
        {
            ESP_LOGI(TAG, "Packet sent successfully");
        }
        uxHighWaterMark2 = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG, "TX task WaterMark2:0x%x", uxHighWaterMark2);
        if (bootup_status)
        {
        	self_test();
        }
        low_power_mode_set_sleep_time(CONFIG_LOW_POWER_MODE_SLEEP_TIME_SEC);
        low_power_mode_enter_deep_sleep();
    }
}

#endif

#if CONFIG_LORA_RECEIVER
static packet_t packet = {0};
static char msg[MSG_BUFFER_SIZE];
void task_rx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start RX");
    UBaseType_t uxHighWaterMark1, uxHighWaterMark2;

    uxHighWaterMark1 = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "RX task WaterMark1:0x%x", uxHighWaterMark1);
    while (1)
    {
        lora_status_t status = lora_receive(&packet);

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
                ESP_LOGE(TAG, "Message CRC error!");
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
                ESP_LOGE(TAG, "Message CRC error!");
                snprintf(msg, sizeof(msg), "{\"data\":\"\"}");
            }
        }

#if CONFIG_ENABLE_MQTT
        char topic[15]={0};
        sprintf(topic, "sensors/%d/data", CONFIG_DEVICE_ID);
  int msg_id = esp_mqtt_client_publish(mqtt_client, topic, msg, 0, 1, 0);
        if (msg_id < 0)
        {
            ESP_LOGE(TAG, "Failed to publish message");
        }
        else
        {
            ESP_LOGI(TAG, "Published message to %s with msg_id: %d", topic, msg_id);
        }

#endif
        uxHighWaterMark2 = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG, "RX WaterMark2:0x%x", uxHighWaterMark2);
    }
}
#endif
