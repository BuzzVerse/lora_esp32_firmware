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
#include "esp_app_desc.h"
#include "i2c.h"
#include "bq27441.h"

/*
 * Definitions, consts
 */
#define TAG "Main"

#define SYS_CLOCK_1SEC (100 / portTICK_PERIOD_MS)
#define SYS_CLOCK_100MS (10 / portTICK_PERIOD_MS)
#define SYS_CLOCK_10MS (1 / portTICK_PERIOD_MS)

#define LORA_INIT_MAX 5				// Num attempts to initialize LoRa

#define xNODE_BME280_FAIL	0x80	// Set when BME280 was not initialized
#define xNODE_LORA_FAIL		0x10	// Set when LoRa init fails
#define xNODE_LOW_BATT		0x08	// Set when low battery detected
#define xNODE_MQTT_FAIL		0x04	// Set when MQTT fails

#define SYS_QUEUE_SZ	10
#define SCH_QUEUE_SZ	10
#define RX_QUEUE_SZ		10

static void initialize_sensors(void)
{
    esp_err_t bme_rc = ESP_OK;
 
    i2c_init();

    bme_rc = bme280_init_driver(CONFIG_BME280_I2C_ADDRESS);
    if (ESP_OK != bme_rc)
    {
        ESP_LOGE(TAG, "BME280 initialization failed");
        // not sure if this should restart, but also not sure what else to do here?
    }
	bme_rc = bme280_set_oversamp(BME280_OVERSAMP_16X, BME280_OVERSAMP_16X, BME280_OVERSAMP_16X);
    bme_rc += bme280_set_settings(STANDBY_10MS, BME280_FILTER_COEFF_16, BME280_NORMAL_MODE);
    if (ESP_OK != bme_rc)
    {
    	ESP_LOGI(TAG, "Failed to set BME280 oversampling/settings!")
    }

    esp_err_t bq_rc = bq27441_init();
    if (bq_rc != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize BQ27441");
        return;
    }

    uint16_t capacity;
	bq_rc = bq27441_read_design_capacity(&capacity);
	if (bq_rc == ESP_OK)
	{
		ESP_LOGI(TAG, "Design Capacity: %d mAh", capacity);
	}
	else
	{
		ESP_LOGE(TAG, "Failed to read design capacity");
	}

	uint8_t soc;
	bq_rc = bq27441_read_soc(&soc);
	if (bq_rc == ESP_OK)
	{
		ESP_LOGI(TAG, "State of Charge: %d %%", soc);
	}
	else
	{
		ESP_LOGE(TAG, "Failed to read state of charge");
	}

	uint16_t voltage;
	bq_rc = bq27441_read_voltage(&voltage);
	if (bq_rc == ESP_OK)
	{
		ESP_LOGI(TAG, "Voltage: %d mV", voltage);
	}
	else
	{
		ESP_LOGE(TAG, "Failed to read voltage");
	}
}

#define SCHEDULE_SZ 3
#define PERIOD_BME280 1
#define PERIOD_BASE PERIOD_BME280			// Set a base for period.
#define PERIOD_STATUS (PERIOD_BME280 * 10) 	// All other periods are multiple of BASE
#define PERIOD_SMS (PERIOD_BME280 * 40)		// All other periods are multiple of BASE

#define MAX_DATA_TYPES 33

#if CONFIG_LORA_RECEIVER
#define MSG_BUFFER_SIZE 128
#endif

// ------------------------------------+
//				N V M                  |
// -----------+---------------+--------+
// NAMESPACE  |  KEY          | VALUE  |
// -----------+---------------+--------+
// radio_link | msg_id        | 8-bit  |
// radio_link | msg_count     | 8-bit  |
// radio_link | comms_err_cnt | 8-bit  |
// radio_link | lowpwr_counter| 16-bit | // TBD: need to update this namespace
// -----------+---------------+--------+

typedef struct nvs_map
{
	const char *namespace;
	const char *key0;			// TBD: This needs to be optimized (loop-through)
	const char *key1;
	const char *key2;
	const char *key3;
} nvs_map_t;

typedef struct bme280
{
	double temp_raw;
	double press_raw;
	double hum_raw;
	uint16_t comms_err_cnt;					// Communication error counter
} bme280_t;

typedef struct bq27441
{
	uint16_t voltage;
} bq27441_t;

typedef struct sensor
{
	bme280_t bme280;
	bq27441_t bq27441;
} sensors_t;

// dummy struct
typedef struct radio
{
	uint8_t msg_counter;
	uint8_t msg_id;
	uint16_t comms_err_cnt;	// Communication error counter
	QueueHandle_t queue_sch;
	QueueHandle_t queue_rx;
} radio_t;

typedef struct system
{
	QueueHandle_t queue;
	uint16_t sleep_time;
	uint16_t life_cycle;
} system_t;

typedef struct node
{
	uint8_t status;  //TBD may need to config of 'node' and status go to system
	system_t system;
	sensors_t sensor;
	radio_t radio;
} node_t;

typedef struct buzzverse_link
{
	void (*handler)(void);
	void (*sender)(void);
} buzzverse_link_t;

typedef struct schedule
{
		DataType type;
		uint32_t period;
} schedule_t;

typedef enum sys_msg
{
	SYS_RUN_LOWPWR_MODE = 0,
	SYS_LORA_FAIL = 1
} sys_msg_t;

typedef enum sch_msg
{
	SCH_START = 0,
	SCH_RADIO_POWER_OFF = 1
} sch_msg_t;



/*
 * Local static vars, functions declarations
 */
static void task_system(void *pvParameters);
static void task_radio_scheduler(void *pvParameters);
static nvs_handle_t sys_mount_nvs(const char *namespace);
static void sys_umount_nvs(nvs_handle_t handle);


/* BuzzVerse link handlers/senders */
static void handler_unknown(void);
static void sender_unknown(void);
static void sender_bme280(void);
static void sender_status(void);
static void sender_sms(void);

/* NVS functions */
static void sys_wr_nvs_from_ram(void);
static void sys_wr_ram_from_nvs(void);

/* Packet construction functions */
static void packet_create_header(packet_t *packet, const DataType type);

#if CONFIG_LORA_RECEIVER
static void task_rx(void *pvParameters);
#endif

static node_t node = {0};
static packet_t packet = {0};

static nvs_map_t nvs_map =
{
		"radio_link",
		"msg_id",
		"msg_counter",
		"comms_err_cnt",
		"life_cycle"
};

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

static schedule_t schedule[SCHEDULE_SZ] =
{
		{ BME280,	PERIOD_BME280 },
		{ STATUS,	PERIOD_STATUS },
		{ SMS,		PERIOD_SMS }
};

/*
 * Local static function definitions
 */

static void handler_unknown(void)
{}

static void packet_create_header(packet_t *packet, const DataType type)
{
	if (NULL != packet)
	{
		packet->version = (CONFIG_PACKET_VERSION << 4); // Version and 4-bit reserved
		packet->id = (CONFIG_CLASS_ID << 4) | CONFIG_DEVICE_ID; // This should be unique
		packet->dataType = type;

		/*
		 * TEMPORARY: update msg counter and ID for debug purpose.
		 */
		packet->msgCount = node.radio.msg_counter++;
		packet->msgID = node.radio.msg_id;
		if (!node.radio.msg_counter)
		{
			node.radio.msg_id++;
		}
	}
	else
	{
		ESP_LOGE(TAG, "Cannot create packet HEADER - null pointer!");
	}
}

static void sender_unknown(void)
{
	packet_create_header(&packet, SMS);
	sprintf((char * restrict)packet.data, "Unknown:0x%x", node.status);
	lora_send(&packet);
}

static void sender_bme280(void)
{
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

    packet_create_header(&packet, BME280);

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
    packet_create_header(&packet, STATUS);

    packet.data[0] = node.status;
    packet.data[1] = (uint8_t)(node.sensor.bq27441.voltage >> 8);
    packet.data[2] = (uint8_t)(node.sensor.bq27441.voltage & 0x00FF);
    packet.data[3] = (uint8_t)(node.system.sleep_time >> 8);
    packet.data[4] = (uint8_t)(node.system.sleep_time & 0x00FF);

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
	const esp_app_desc_t *esp_app_desc = esp_app_get_description();

    packet_create_header(&packet, SMS);

	snprintf((char *restrict)packet.data, DATA_SIZE, "FW:%s B:%d[mV] S:%d",
			esp_app_desc->version,
			node.sensor.bq27441.voltage,
			node.system.sleep_time);

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

#if CONFIG_LORA_RECEIVER && CONFIG_ENABLE_MQTT

const esp_mqtt_client_config_t default_mqtt_cfg = { 0 };

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



static void initialize_sensors(void)
{
    esp_err_t bme_rc = ESP_OK;

	bme_rc = bme280_init_driver(CONFIG_BME280_I2C_ADDRESS);
	if (ESP_OK == bme_rc) {

		bme_rc += bme280_set_oversamp(BME280_OVERSAMP_16X, BME280_OVERSAMP_16X, BME280_OVERSAMP_16X);
		bme_rc += bme280_set_settings(STANDBY_10MS, BME280_FILTER_COEFF_16, BME280_NORMAL_MODE);
	}

	if (ESP_OK != bme_rc)
	{
		ESP_LOGE(TAG, "BME280 initialization failed");
		node.status |= xNODE_BME280_FAIL;
	}
}

/*
 * System task responsible for handling system wide events.
 */
static void task_system(void *pvParameters)
{
	ESP_LOGI(TAG, "Started task:%s", pcTaskGetName(NULL));

	sys_msg_t sys_msg;
	node.system.sleep_time = CONFIG_LOW_POWER_MODE_SLEEP_TIME_SEC;
	sys_wr_ram_from_nvs();
    //initialize_sensors();

    while (1)
	{
		xQueueReceive(node.system.queue, &sys_msg, portMAX_DELAY);
		switch (sys_msg)
		{
			case SYS_RUN_LOWPWR_MODE:
				node.system.life_cycle++;	// Update life cycle
				sys_wr_nvs_from_ram();
				low_power_mode_set_sleep_time(CONFIG_LOW_POWER_MODE_SLEEP_TIME_SEC);
				low_power_mode_enter_deep_sleep();
				break;
			case SYS_LORA_FAIL:
		        node.status |= xNODE_LORA_FAIL;
				break;
			default:
				break;
		}
	}
}

#if CONFIG_LORA_TRANSMITTER


void task_radio_scheduler(void *pvParameters)
{
	ESP_LOGI(TAG, "Started task:%s", pcTaskGetName(NULL));
	sch_msg_t sch_msg = SCH_START;
	sys_msg_t sys_msg = SYS_LORA_FAIL;
	uint8_t i;

    if (LORA_OK != lora_init())
    {
        ESP_LOGE(TAG, "LoRa initialization failed. Retrying...");

        for (i = 0; i < LORA_INIT_MAX && (LORA_OK != lora_init()); i++);

        if (LORA_INIT_MAX == i)
        {
			xQueueSend(node.system.queue, (void *)&sys_msg, 0);
			ESP_LOGE(TAG, "LoRa initialization failed. Deleting task");
			vTaskDelete(NULL);
        }
    }

	xQueueSend(node.radio.queue_sch,(const void *) &sch_msg, 0);
	while (1)
	{
		xQueueReceive(node.radio.queue_sch, &sch_msg, portMAX_DELAY);
		switch (sch_msg)
		{
			case SCH_START:
				for (i = 0; i < SCHEDULE_SZ; i++)
				{
					if (!(node.system.life_cycle % schedule[i].period)) // Call it life cycle :) ?
					{
						buzzverse_link[schedule[i].type].sender();
					}
				}
				sys_msg_t sys_msg = SYS_RUN_LOWPWR_MODE;
				xQueueSend(node.system.queue, (void *)&sys_msg, SYS_CLOCK_1SEC);
				break;
			default:
				ESP_LOGE(TAG, "Unknown sch_msg:%d", sch_msg);
				break;
		}
	}
}

#endif

#if CONFIG_LORA_RECEIVER

static char msg[MSG_BUFFER_SIZE];
void task_rx(void *pvParameters)
{
	ESP_LOGI(TAG, "Started task:%s", pcTaskGetName(NULL));
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

            if (LORA_OK != status)
            {
                ESP_LOGE(TAG, "Message CRC error!");
            }
		snprintf(msg, sizeof(msg), "{\"BME280\":{\"temperature\":%.2f, \"pressure\":%.2f, \"humidity\":%.2f}}",
				received_temp, received_press, received_hum);
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
        char topic[20]={0};
        sprintf(topic, "sensors/%d/data", CONFIG_DEVICE_ID);
  int msg_id = esp_mqtt_client_publish(mqtt_client, topic, msg, 0, 1, 0);
        if (msg_id < 0)
        {
            ESP_LOGE(TAG, "Failed to publish message");
        }
        else
        {
            ESP_LOGI(TAG, "Published message to %s with msg_id: %d", topic, msg_id);
            ESP_LOGI(TAG, "message:%s", msg);
        }

#endif
        uxHighWaterMark2 = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG, "RX WaterMark2:0x%x", uxHighWaterMark2);
    }
}
<<<<<<< HEAD
#endif

static nvs_handle_t sys_mount_nvs(const char *namespace)
{
	nvs_handle_t handle;
	esp_err_t err = nvs_flash_init();

	if (NULL == namespace)
	{
		ESP_LOGE(TAG, "Invalid namespace NULL");
		return 0;
	}

	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		// NVS partition was truncated and needs to be erased
		// Retry nvs_flash_init
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
		if (ESP_OK != err)
		{
			ESP_LOGE(TAG, "FATAL could not recover flash!");
			return 0;
		}
	}
	// Open
	ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle... ");
	err = nvs_open(namespace, NVS_READWRITE, &handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
	    nvs_close(handle);
	    fflush(stdout);
	} else {
		ESP_LOGI(TAG, "Done");
	}
	return handle;
}

static void sys_umount_nvs(nvs_handle_t handle)
{
	esp_err_t err;
    // Commit written values.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    ESP_LOGI(TAG, "Committing updates in NVS");
    err = nvs_commit(handle);
    ESP_LOGI(TAG, "%s", (err != ESP_OK) ? "Failed!" : "Done");

    // Close
    nvs_close(handle);
    fflush(stdout);
}

static void sys_wr_nvs_from_ram(void)
{
	nvs_handle_t handle;

    handle = sys_mount_nvs(nvs_map.namespace);

    nvs_set_u8(handle, nvs_map.key0, node.radio.msg_counter);
    nvs_set_u8(handle, nvs_map.key1, node.radio.msg_id);
   	nvs_set_u16(handle, nvs_map.key2, node.sensor.bme280.comms_err_cnt);
   	nvs_set_u16(handle, nvs_map.key3, node.system.life_cycle);

    sys_umount_nvs(handle);
}

static void sys_wr_ram_from_nvs(void)
{
	nvs_handle_t handle;

    handle = sys_mount_nvs(nvs_map.namespace);

    nvs_get_u8(handle, nvs_map.key0, &node.radio.msg_counter);
    nvs_get_u8(handle, nvs_map.key1, &node.radio.msg_id);
    nvs_get_u16(handle, nvs_map.key2, &node.sensor.bme280.comms_err_cnt);
   	nvs_get_u16(handle, nvs_map.key3, &node.system.life_cycle);

    sys_umount_nvs(handle);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing LoRa");


    node.system.queue = xQueueCreate(SYS_QUEUE_SZ, sizeof(sys_msg_t));
    if (NULL != node.system.queue)
    {
    	xTaskCreate(&task_system, "SYSTEM", 1024 * 4, NULL, 5, NULL);
    }
    else
    {
    	ESP_LOGE(TAG, "SYSTEM task not created!");
    }

#if CONFIG_LORA_TRANSMITTER

    node.radio.queue_sch = xQueueCreate(SCH_QUEUE_SZ, sizeof(sch_msg_t));
    if (NULL != node.radio.queue_sch)
    {
    	xTaskCreate(&task_radio_scheduler, "RADIO_SCH", 1024 * 4, NULL, 5, NULL);
    }
    else
    {
    	ESP_LOGE(TAG, "RADIO_SCH task not craeted!");
    }
#endif
    lora_init();
#if CONFIG_LORA_RECEIVER

#if CONFIG_ENABLE_MQTT
    initialize_mqtt();
#endif

    node.radio.queue_rx = xQueueCreate(RX_QUEUE_SZ, sizeof(uint8_t *));
    xTaskCreate(&task_rx, "RADIO_RX", 1024 * 4, NULL, 5, NULL);
#endif
}


