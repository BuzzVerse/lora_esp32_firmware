// lora.c

#include "lora.h"
#include "driver/lora_driver.h"
#include "protocols/packet/packet.h"
#include "api/driver_api.h"

#include <string.h>
#include <stdio.h>

// Remove if wanting to make lora.c HAL independent
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#define LORA_FREQUENCY (CONFIG_LORA_FREQUENCY * 1e6)
#define LORA_CODING_RATE CONFIG_LORA_CODING_RATE
#define LORA_BANDWIDTH CONFIG_LORA_BANDWIDTH
#define LORA_SPREADING_FACTOR CONFIG_LORA_SPREADING_FACTOR
#define LORA_CRC CONFIG_LORA_CRC
#define LORA_POWER CONFIG_TX_POWER

#define SEND_TIMEOUT 15000        // Adjusted to milliseconds
#define CONFIRMATION_TIMEOUT 5000 // Adjusted to milliseconds

TimerHandle_t sendTimer;
volatile bool timeout_occurred = false;

TaskHandle_t sendTaskHandle = NULL;
TaskHandle_t mainTaskHandle = NULL;

void pack_packet(uint8_t *buffer, packet_t *packet);
void print_buffer(uint8_t *buffer, size_t size);
size_t get_packet_size(DataType type);

void lora_send_task(void *pvParameters)
{
    if (pvParameters == NULL)
    {
        ESP_LOGE(LORA_TAG, "pvParameters is NULL in lora_send_task.\n");
        return;
    }

    packet_t *packet = (packet_t *)pvParameters;
    uint8_t buffer[PACKET_SIZE] = {0};
    size_t packet_size = get_packet_size(packet->dataType) + META_DATA_SIZE;

    pack_packet(buffer, packet);

    print_buffer(buffer, packet_size);

    // Send the packet using the HAL function
    lora_send_packet(buffer, packet_size);

    // Notify the main task of completion
    xTaskNotifyGive(mainTaskHandle);

    // Terminate the task
    vTaskDelete(NULL);
}

void pack_packet(uint8_t *buffer, packet_t *packet)
{
    if (NULL == buffer || NULL == packet)
    {
        ESP_LOGE(LORA_TAG, "Buffer or packet is NULL.\n");
        return;
    }

    buffer[PACKET_VERSION_IDX] = packet->version;
    buffer[PACKET_ID_IDX] = packet->id;
    buffer[PACKET_MSG_ID_IDX] = packet->msgID;
    buffer[PACKET_MSG_COUNT_IDX] = packet->msgCount;
    buffer[PACKET_DATA_TYPE_IDX] = packet->dataType;
    memcpy(&buffer[META_DATA_SIZE], packet->data, DATA_SIZE);
}

void print_buffer(uint8_t *buffer, size_t size)
{
    if (NULL == buffer)
    {
        ESP_LOGE(LORA_TAG, "Buffer is NULL.\n");
        return;
    }

    printf("BUFFER: \n");
    for (size_t i = 0; i < size; i++)
    {
        printf("0x%x ", buffer[i]);
    }
    printf("\n");
}

void sendPacketTimeoutHandler(TimerHandle_t xTimer)
{
    ESP_LOGE(LORA_TAG, "Send packet timeout.\n");
    timeout_occurred = true;

    if (sendTaskHandle != NULL)
    {
        vTaskDelete(sendTaskHandle);
    }

    lora_sleep_mode();
    lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

    xTaskNotifyGive(mainTaskHandle);
}

void noConfirmationHandler(TimerHandle_t xTimer)
{
    ESP_LOGE(LORA_TAG, "Confirmation timeout.\n");
    timeout_occurred = true;
    xTimerStop(xTimer, 0);
}

lora_status_t lora_init(void)
{
    ESP_LOGI(LORA_TAG, "FREQ: %f", LORA_FREQUENCY);
    ESP_LOGI(LORA_TAG, "CODING RATE: %d", LORA_CODING_RATE);
    ESP_LOGI(LORA_TAG, "BANDWIDTH: %d", LORA_BANDWIDTH);
    ESP_LOGI(LORA_TAG, "SPREADING FACTOR: %d", LORA_SPREADING_FACTOR);
    ESP_LOGI(LORA_TAG, "CRC: %d", LORA_CRC);
    ESP_LOGI(LORA_TAG, "TRANSMIT POWER: %d", LORA_POWER);

    if (LORA_OK != lora_driver_init())
    {
        ESP_LOGE(LORA_TAG, "LoRa initialization failed\n");
        esp_restart();
    }

    lora_set_frequency(LORA_FREQUENCY);
    lora_set_tx_power(LORA_POWER);

    if (LORA_CRC)
    {
        lora_enable_crc();
    }
    lora_set_coding_rate(LORA_CODING_RATE);
    lora_set_bandwidth(LORA_BANDWIDTH);
    lora_set_spreading_factor(LORA_SPREADING_FACTOR);

    return LORA_OK;
}

lora_status_t lora_send(packet_t *packet)
{
    if (NULL == packet)
    {
        ESP_LOGE(LORA_TAG, "Buffer or packet is NULL.\n");
        return LORA_FAILED_SEND_PACKET;
    }

    sendTimer = xTimerCreate("SendTimer", pdMS_TO_TICKS(SEND_TIMEOUT), pdFALSE, (void *)0, sendPacketTimeoutHandler);

    if (NULL == sendTimer)
    {
        ESP_LOGE(LORA_TAG, "Failed to create timer.\n");
        return LORA_FAILED_SEND_PACKET;
    }

    mainTaskHandle = xTaskGetCurrentTaskHandle();

    BaseType_t xReturned = xTaskCreate(lora_send_task, "LoRaSendTask", 2048, (void *)packet, tskIDLE_PRIORITY, &sendTaskHandle);

    if (xReturned != pdPASS)
    {
        ESP_LOGE(LORA_TAG, "Failed to create send task.\n");
        return LORA_FAILED_SEND_PACKET;
    }

    if (xTimerStart(sendTimer, 0) != pdPASS)
    {
        ESP_LOGE(LORA_TAG, "Failed to start timer.\n");
        return LORA_FAILED_SEND_PACKET;
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    xTimerStop(sendTimer, 0);

    if (timeout_occurred)
    {
        return LORA_FAILED_SEND_PACKET;
    }

    return LORA_OK;
}

lora_status_t lora_receive(packet_t *packet)
{
    if (NULL == packet)
    {
        ESP_LOGE(LORA_TAG, "Packet is NULL.\n");
        return LORA_FAILED_RECEIVE_PACKET;
    }

    uint8_t buffer[PACKET_SIZE] = {0};
    uint8_t length = 0;
    uint8_t rssi = 0;
    uint8_t snr = 0;

    lora_receive_mode();

    while (1)
    {
        bool hasReceived = false;
        bool crc_error = false;

        lora_received(&hasReceived, &crc_error);

        if (hasReceived)
        {
            lora_receive_packet(buffer, &length, sizeof(buffer));

            lora_packet_rssi(&rssi);
            lora_packet_snr(&snr);

            print_buffer(buffer, sizeof(buffer));

            ESP_LOGE(LORA_TAG, "RSSI: %d", rssi);
            ESP_LOGE(LORA_TAG, "SNR: %d", snr);

            packet->version = buffer[PACKET_VERSION_IDX];
            packet->id = buffer[PACKET_ID_IDX];
            packet->msgID = buffer[PACKET_MSG_ID_IDX];
            packet->msgCount = buffer[PACKET_MSG_COUNT_IDX];
            packet->dataType = buffer[PACKET_DATA_TYPE_IDX];

            memcpy(packet->data, &buffer[META_DATA_SIZE], DATA_SIZE);
            return crc_error ? LORA_CRC_ERROR : LORA_OK;
        }
        lora_delay(LORA_DELAY_20MS);
    }
}

size_t get_packet_size(DataType type)
{
    switch (type)
    {
    case BME280:
        return 3; // 1B temp, 1B hum, 1B pres
    case BMA400:
        return 24; // 8B x-axis, 8B y-axis, 8B z-axis
    case MQ2:
        return 17; // 1B gas type, 16B value
    case GPS:
        return 16; // 8B longitude, 8B latitude
    case SMS:
        return 59; // Max 59B String
    default:
        return 0; // Unsupported type
    }
}
