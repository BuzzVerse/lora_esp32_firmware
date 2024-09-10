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
#include "nvs_flash.h"
#include "nvs.h"

#define TAG "Main"

#define MAX_DATA_TYPES 33

typedef struct bme280
{
	double temp_raw;
	double press_raw;
	double hum_raw;
	uint16_t comms_err_cnt;					// Communication error counter
} bme280_t;

typedef struct sensor
{
	bme280_t bme280;
} sensor_t;

// dummy struct
typedef struct batt
{
	uint8_t cap;			// Capacity in % range 0-100%
} batt_t;

// dummy struct
typedef struct radio
{
	uint8_t	bw;				// Bandwidth
	uint8_t sf;				// Spreading factor
	uint8_t cr;				// Coding rate
	uint16_t comms_err_cnt;	// Communication error counter
} radio_t;

typedef struct node
{
	uint8_t status;
	uint8_t batt_cap;
	sensor_t sensor;
	radio_t radio;
	batt_t batt;
} node_t;

//#define CONFIG_LORA_TRANSMITTER 1
#define xNODE_BME280_FAIL	0x80	// Set when BME280 was not initialized
#define xNODE_LORA_FAIL		0x10	// Set when LoRa init fails
#define xNODE_LOW_BATT		0x08	// Set when low battery detected
#define xNODE_MQTT_FAIL		0x04	// Set when MQTT fails

static node_t node = {0};
static packet_t packet = {0};

static nvs_handle_t nvs_mount(void);
static void nvs_umount(nvs_handle_t handle);
static void self_test(void);

typedef struct buzzverse_link
{
	void (*handler)(void);
	void (*sender)(void);
} buzzverse_link_t;

static void handler_unknown(void)
{}

static void sender_unknown(void)
{
    packet.version = (CONFIG_PACKET_VERSION << 4); 			// Reserved 4 bits set to 0
    packet.id = (CONFIG_CLASS_ID << 4) | CONFIG_DEVICE_ID;
    packet.dataType = SMS;
	sprintf((char * restrict)packet.data, "NODE STATUS:0x%x", node.status);
	lora_send(&packet);
}

static void sender_bme280(void)
{
	nvs_handle_t handle;
	uint16_t cnt;

    if (ESP_FAIL == bme280_read_temperature(&node.sensor.bme280.temp_raw))
    {
    	node.sensor.bme280.comms_err_cnt++;
    }
    if (ESP_FAIL == bme280_read_pressure(&node.sensor.bme280.press_raw))
    {
    	node.sensor.bme280.comms_err_cnt++;
    }
    if (ESP_FAIL == bme280_read_humidity(&node.sensor.bme280.hum_raw))
    {
    	node.sensor.bme280.comms_err_cnt++;
    }

    // Fill the packet with the meta data
    packet.version = (CONFIG_PACKET_VERSION << 4); 			// Reserved 4 bits set to 0
    packet.id = (CONFIG_CLASS_ID << 4) | CONFIG_DEVICE_ID;
    packet.dataType = CONFIG_DATA_TYPE;

    handle = nvs_mount();
    nvs_get_u8(handle, "msgCount", &packet.msgCount);
    nvs_get_u8(handle, "msgID", &packet.msgID);
    nvs_get_u16(handle, "bme280.comms_err_cnt", &cnt);

    ESP_LOGI(TAG, "msgID:%d msgCount:%d", packet.msgID, packet.msgCount);

    if (0xFF == packet.msgCount++)
    {
    	packet.msgID++;
    }
    nvs_set_u8(handle, "msgID", packet.msgID);
    nvs_set_u8(handle, "msgCount", packet.msgCount);
    if (cnt != node.sensor.bme280.comms_err_cnt)
    {
    	node.sensor.bme280.comms_err_cnt = cnt;
    	nvs_set_u16(handle, "msgCount", cnt);
    }
    nvs_umount(handle);

    // Convert raw sensor readings
	packet.data[0] = (int8_t)(node.sensor.bme280.temp_raw * 2);             // Scale temperature for higher precision and fit into int8_t
	packet.data[1] = (int8_t)((node.sensor.bme280.press_raw / 100) - 1000); // Convert Pa to hPa, subtract 1000
	packet.data[2] = (uint8_t)node.sensor.bme280.hum_raw;                   // Fit humidity into uint8_t

	// Log the packet data before sending
	ESP_LOGI(pcTaskGetName(NULL), "Temperature: %.2f C (scaled to %d)", node.sensor.bme280.temp_raw, packet.data[0]);
	ESP_LOGI(pcTaskGetName(NULL), "Pressure: %.2f hPa (stored as %d)", node.sensor.bme280.press_raw / 100, packet.data[1]);
	ESP_LOGI(pcTaskGetName(NULL), "Humidity: %.2f %% (stored as %u)", node.sensor.bme280.hum_raw, packet.data[2]);
    lora_status_t send_status = lora_send(&packet);

    if (LORA_OK != send_status)
    {
        ESP_LOGE(TAG, "Packet send failed");
    }
    else
    {
        ESP_LOGI(TAG, "Packet sent successfully");
    }
}

static void sender_status(void)
{
	nvs_handle_t handle;

    packet.version = (CONFIG_PACKET_VERSION << 4); 			// Reserved 4 bits set to 0
    packet.id = (CONFIG_CLASS_ID << 4) | CONFIG_DEVICE_ID;
	packet.dataType = STATUS;

    handle = nvs_mount();
    nvs_get_u8(handle, "msgCount", &packet.msgCount);
    nvs_get_u8(handle, "msgID", &packet.msgID);
    if (0xFF == packet.msgCount++)
    {
    	packet.msgID++;
    }
    nvs_set_u8(handle, "msgID", packet.msgID);
    nvs_set_u8(handle, "msgCount", packet.msgCount);
    nvs_umount(handle);

    packet.data[0] = node.status;
    packet.data[1] = node.batt.cap;


	ESP_LOGI(pcTaskGetName(NULL), "Data: %s", packet.data);
    lora_status_t send_status = lora_send(&packet);

    if (LORA_OK != send_status)
    {
        ESP_LOGE(TAG, "Packet send failed");
    }
    else
    {
        ESP_LOGI(TAG, "Packet sent successfully");
    }
}

static void sender_sms(void)
{
    // Fill the packet with the meta data
    packet.version = (CONFIG_PACKET_VERSION << 4); 			// Reserved 4 bits set to 0
    packet.id = (CONFIG_CLASS_ID << 4) | CONFIG_DEVICE_ID;
    packet.dataType = CONFIG_DATA_TYPE;
	packet.dataType = SMS;

    if (node.status)
    {
		sprintf((char * restrict)packet.data, "NODE STATUS:0x%x", node.status);
    }
    else
    {
    	sprintf((char * restrict)packet.data, "System booted-up");
    }
	ESP_LOGI(pcTaskGetName(NULL), "Data: %s", packet.data);
    lora_status_t send_status = lora_send(&packet);

    if (LORA_OK != send_status)
    {
        ESP_LOGE(TAG, "Packet send failed");
    }
    else
    {
        ESP_LOGI(TAG, "Packet sent successfully");
    }
}

static buzzverse_link_t buzzverse_link[MAX_DATA_TYPES] =
{
		{	&handler_unknown, 	&sender_unknown 	},	// 0 - Reserved
		{	&handler_unknown,	&sender_bme280		},	// 1 - BME280
		{	&handler_unknown,	&sender_unknown		},	// 2 - BMA400
		{	&handler_unknown,	&sender_unknown		},	// 3 - MQ2
		{	&handler_unknown,	&sender_unknown		},	// 4 - GPS
		{	&handler_unknown,	&sender_status		},	// 5 - STATUS
		{	&handler_unknown,	&sender_unknown		},	// 6
		{	&handler_unknown,	&sender_unknown		},	// 7
		{	&handler_unknown,	&sender_unknown		},	// 8
		{	&handler_unknown,	&sender_unknown		},	// 9
		{	&handler_unknown,	&sender_unknown		},	// 10
		{	&handler_unknown,	&sender_unknown		},	// 11
		{	&handler_unknown,	&sender_unknown		},	// 12
		{	&handler_unknown,	&sender_unknown		},	// 13
		{	&handler_unknown,	&sender_unknown		},	// 14
		{	&handler_unknown,	&sender_unknown		},	// 15
		{	&handler_unknown,	&sender_unknown		},	// 16
		{	&handler_unknown,	&sender_unknown		},	// 17
		{	&handler_unknown,	&sender_unknown		},	// 18
		{	&handler_unknown,	&sender_unknown		},	// 19
		{	&handler_unknown,	&sender_unknown		},	// 20
		{	&handler_unknown,	&sender_unknown		},	// 21
		{	&handler_unknown,	&sender_unknown		},	// 22
		{	&handler_unknown,	&sender_unknown		},	// 23
		{	&handler_unknown,	&sender_unknown		},	// 24
		{	&handler_unknown,	&sender_unknown		},	// 25
		{	&handler_unknown,	&sender_unknown		},	// 26
		{	&handler_unknown,	&sender_unknown		},	// 27
		{	&handler_unknown,	&sender_unknown		},	// 28
		{	&handler_unknown,	&sender_unknown		},	// 29
		{	&handler_unknown,	&sender_unknown		},	// 30
		{	&handler_unknown,	&sender_unknown		},	// 31
		{	&handler_unknown,	&sender_sms			}	// 32 - SMS
};
// LOCAL



#if CONFIG_LORA_TRANSMITTER
static void task_tx(void *pvParameters);

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
        node.status |= xNODE_BME280_FAIL;
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
        node.status = xNODE_LORA_FAIL;
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

void task_tx(void *pvParameters)
{
    ESP_LOGI(TAG, "Started task:%s", pcTaskGetName(NULL));
    vTaskDelay(500 / portTICK_PERIOD_MS);

    while (1)
    {
        buzzverse_link[CONFIG_DATA_TYPE].sender();

        if (node.status & xNODE_BME280_FAIL)
        {
        	buzzverse_link[STATUS].sender();
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

static void self_test(void)
{
	ESP_LOGI(TAG, "--SELF TEST BEGIN--");
	if (node.status & xNODE_BME280_FAIL)
	{
		ESP_LOGI(TAG, "--BME280 FAIL");
		ESP_LOGI(TAG, "--Initializing BME280 driver: %s", ESP_OK == bme280_init_driver(CONFIG_BME280_I2C_ADDRESS)?"OK":"FAILED");
	}
	if (node.status & xNODE_LORA_FAIL)
	{
		ESP_LOGI(TAG, "--LORA FAIL");
		ESP_LOGI(TAG, "--Initializing LoRa driver: %s", LORA_OK == lora_init()?"OK":"FAILED");
	}
	ESP_LOGI(TAG, "--SELF TEST END--");
}

static nvs_handle_t nvs_mount(void)
{
	nvs_handle_t handle;
	esp_err_t err = nvs_flash_init();

	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		// NVS partition was truncated and needs to be erased
		// Retry nvs_flash_init
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	// Open
	ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle... ");
	err = nvs_open("storage", NVS_READWRITE, &handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	    nvs_close(handle);
	    fflush(stdout);
	} else {
		ESP_LOGI(TAG, "Done\n");
	}
	return handle;
}

static void nvs_umount(nvs_handle_t handle)
{
	esp_err_t err;
    // Commit written values.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    ESP_LOGI(TAG, "Committing updates in NVS ... ");
    err = nvs_commit(handle);
    ESP_LOGI(TAG, "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Close
    nvs_close(handle);
    fflush(stdout);
}
