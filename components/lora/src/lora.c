// lora.c

#include "lora.h"
#include "driver/lora_driver.h"
#include "api/driver_api.h"
#include <string.h>
#include <stdio.h>

#define LORA_FREQUENCY (CONFIG_LORA_FREQUENCY * 1e6)
#define LORA_CODING_RATE CONFIG_LORA_CODING_RATE
#define LORA_BANDWIDTH CONFIG_LORA_BANDWIDTH
#define LORA_SPREADING_FACTOR CONFIG_LORA_SPREADING_FACTOR
#define LORA_CRC CONFIG_LORA_CRC

lora_status_t lora_init(void)
{

    printf("FREQ: %f\n", LORA_FREQUENCY);
    printf("CODING RATE: %d\n", LORA_CODING_RATE);
    printf("BANDWIDTH: %d\n", LORA_BANDWIDTH);
    printf("SPREADING FACTOR: %d\n", LORA_SPREADING_FACTOR);
    printf("CRC: %d\n", LORA_CRC);

    if (LORA_OK != lora_driver_init())
    {
        printf("LoRa initialization failed\n");
        return LORA_FAILED_INIT;
    }

    lora_set_frequency(LORA_FREQUENCY);
    if (LORA_CRC)
    {
        lora_enable_crc();
    }
    lora_set_coding_rate(LORA_CODING_RATE);
    lora_set_bandwidth(LORA_BANDWIDTH);
    lora_set_spreading_factor(LORA_SPREADING_FACTOR);

    lora_dump_registers();
    return LORA_OK;
}

lora_status_t lora_send(const lora_packet_t *packet)
{
    // Buffer to hold the received packet
    uint8_t buffer[PACKET_SIZE] = {0};

    // Pack the id and version fields
    buffer[0] = packet->version;
    buffer[1] = packet->id;
    buffer[2] = packet->msgID;
    buffer[3] = packet->msgCount;
    buffer[4] = packet->dataType;

    // Copy the data
    memcpy(&buffer[META_DATA_SIZE], packet->data, DATA_SIZE);

    printf("\n");
    for (int i = 0; i < sizeof(buffer); i++)
    {
        printf("0x%x ", buffer[i]);
    }
    printf("\n");

    return lora_send_packet(buffer, sizeof(buffer));
}

lora_status_t lora_send_confirmation(const lora_packet_t *packet)
{
    lora_packet_t receive_packet;

    lora_send(packet);
    for (int i = 0; i < 5; i++)
    {
        lora_receive(&receive_packet);
        if (receive_packet.msgID == packet->msgID)
        {
            ESP_LOGI(LORA_TAG, "Received confirmation for message ID %d", packet->msgID);
            return LORA_OK;
        }
    }

    ESP_LOGE(LORA_TAG, "Could not receive confirmation for message ID %d", packet->msgID);
    return LORA_FAILED_RECEIVE_PACKET;
}

lora_status_t lora_receive(lora_packet_t *packet)
{
    // Buffer to hold the received packet
    uint8_t buffer[PACKET_SIZE] = {0};
    uint8_t length = 0;

    lora_receive_mode(); // Put into receive mode

    while (1)
    {
        bool hasReceived = false;
        lora_received(&hasReceived);

        if (hasReceived)
        {
            lora_receive_packet(buffer, &length, sizeof(buffer));

            for (int i = 0; i < sizeof(buffer); i++)
            {
                printf("0x%x ", buffer[i]);
            }
            printf("\n");

            // Unpack the id and version fields
            packet->version = buffer[0];
            packet->id = buffer[1];
            packet->msgID = buffer[2];
            packet->msgCount = buffer[3];
            packet->dataType = buffer[4];

            // Copy the data
            memcpy(packet->data, &buffer[META_DATA_SIZE], DATA_SIZE);
            return LORA_OK;
        }
        lora_delay(2);
    }
}

lora_status_t lora_receive_confirmation(lora_packet_t *packet)
{
    
}