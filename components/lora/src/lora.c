// lora.c

#include "lora.h"
#include "driver/lora_driver.h"
#include "api/driver_api.h"
#include <string.h>
#include <stdio.h>

#define LORA_FREQUENCY 433e6
#define LORA_CODING_RATE 8
#define LORA_BANDWIDTH 2
#define LORA_SPREADING_FACTOR 12
#define LORA_CRC 1

void lora_init(void)
{
    if (LORA_OK != lora_driver_init())
    {
        printf("LoRa initialization failed");
        return;
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
}

void lora_send(const LoRaPacket *packet)
{
    uint8_t buffer[64] = {0}; // Ensure buffer is 64 bytes

    // Pack the id and version fields
    buffer[0] = (packet->id & 0xF0) | ((packet->version >> 4) & 0x0F);
    buffer[1] = packet->msgID;
    buffer[2] = packet->msgCount;
    buffer[3] = packet->dataType;

    // Copy the data
    memcpy(&buffer[4], packet->data, DATA_SIZE);

    for (int i = 0; i < 64; i++)
    {
        printf("0x%x \n", buffer[i]);
    }

    lora_send_packet(buffer, 8);
}

void lora_receive(LoRaPacket *packet)
{
    uint8_t buffer[64] = {0};
    uint8_t length = 0;

    while (1)
    {
        lora_receive_mode(); // Put into receive mode
        bool hasReceived = false;
        lora_received(&hasReceived);

        if (hasReceived)
        {
            lora_receive_packet(buffer, &length, sizeof(buffer));

            // Unpack the id and version fields
            packet->id = buffer[0] >> 4;
            packet->version = (buffer[0] & 0x0F) << 4;
            packet->msgID = buffer[1];
            packet->msgCount = buffer[2];
            packet->dataType = buffer[3];

            // Copy the data
            memcpy(packet->data, &buffer[4], DATA_SIZE);
        }

        lora_delay(2);
    }
}
