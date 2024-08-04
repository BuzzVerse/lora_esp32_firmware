#include "bluetooth.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_log.h"
#include "bme280.h"
#include "lora.h"

const static char *TAG = "bluetooth_manager";

typedef struct
{
    double latitude;
    double longitude;
} location_data_t;

typedef struct
{
    double temperature;
    double pressure;
    double humidity;
} sensor_data_t;

static sensor_data_t current_sensor_data;

uint8_t ble_addr_type;
void ble_app_advertise(void);

ble_uuid16_t device_uuid;

void host_task(void *param)
{
    nimble_port_run();
}

static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->om->om_len == sizeof(location_data_t))
    {
        location_data_t location_data;
        memcpy(&location_data, ctxt->om->om_data, sizeof(location_data_t));

        printf("Latitude: %f\n", location_data.latitude);
        printf("Longitude: %f\n", location_data.longitude);

        packet_t packet = {0};
        packet.version = (CONFIG_PACKET_VERSION << 4) | 0; // Reserved 4 bits set to 0
        packet.id = (CONFIG_CLASS_ID << 4) | CONFIG_DEVICE_ID;
        packet.msgID = 1;                   // Example message ID
        packet.msgCount = 1;                // Example message count (optional, set as needed)
        packet.dataType = CONFIG_DATA_TYPE; // Example data type

        memcpy(packet.data, &location_data, sizeof(location_data_t));

        printf("Packet size: %d\n", sizeof(packet));

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
    else
    {
        printf("Received data size mismatch\n");
    }

    return 0;
}

static int sensor_read(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    double temp_raw, press_raw, hum_raw;
    bme280_read_temperature(&temp_raw);
    bme280_read_pressure(&press_raw);
    bme280_read_humidity(&hum_raw);

    current_sensor_data.temperature = temp_raw;
    current_sensor_data.pressure = press_raw / 100;
    current_sensor_data.humidity = hum_raw;

    ESP_LOGI(pcTaskGetName(NULL), "Temperature: %.2f C", temp_raw);
    ESP_LOGI(pcTaskGetName(NULL), "Pressure: %.2f hPa", press_raw / 100);
    ESP_LOGI(pcTaskGetName(NULL), "Humidity: %.2f %%", hum_raw);

    os_mbuf_append(ctxt->om, &current_sensor_data, sizeof(current_sensor_data));
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = (const ble_uuid_t *) &device_uuid,
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xCAFE), // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
         {.uuid = BLE_UUID16_DECLARE(0xBEEF), // UUID for sensor data
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = sensor_read},
         {0}}},
    {0}};

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]){ device_uuid };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

void init_uuids() {
    uint16_t uuid16 = (uint16_t)strtol(CONFIG_BT_DEVICE_UUID, NULL, 16);
    device_uuid.u.type = BLE_UUID_TYPE_16;
    device_uuid.value = uuid16;
}

void init_ble()
{
    init_uuids();
    ESP_LOGI(TAG, "Initialising Bluetooth");
    nimble_port_init();
    ble_svc_gap_device_name_set(CONFIG_BT_DEVICE_NAME);
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);
}