// lora.c

#include "lora.h"
#include "driver/lora_driver.h"
#include "protocols/packet.h"
#include "api/driver_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <string.h>
#include <stdio.h>

#define LORA_FREQUENCY (CONFIG_LORA_FREQUENCY * 1e6)
#define LORA_CODING_RATE CONFIG_LORA_CODING_RATE
#define LORA_BANDWIDTH CONFIG_LORA_BANDWIDTH
#define LORA_SPREADING_FACTOR CONFIG_LORA_SPREADING_FACTOR
#define LORA_CRC CONFIG_LORA_CRC

#define CONFIRMATION_TIMEOUT 5

TimerHandle_t xTimer;
volatile bool timeout_occurred = false;

void noConfirmationHandler(TimerHandle_t xTimer)
{
    printf("Confirmation timeout.\n");
    timeout_occurred = true;
}

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

    return LORA_OK;
}

lora_status_t lora_send(packet_t *packet)
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

lora_status_t lora_send_confirmation(packet_t *packet)
{
    packet_t receive_packet;
    lora_status_t status;

    xTimer = xTimerCreate("Timer", pdMS_TO_TICKS(8000), pdFALSE, (void *)0, noConfirmationHandler);

    if (xTimer == NULL)
    {
        // Timer creation failed
        printf("Failed to create timer.\n");
        return LORA_FAILED_SEND_PACKET;
    }

    // Send the packet
    status = lora_send(packet);
    if (status != LORA_OK)
    {
        printf("Failed to send message ID %d\n", packet->msgID);
        return status;
    }
    printf("Sent message ID %d\n", packet->msgID);

    // Wait for confirmation
    printf("Waiting for confirmation for message ID %d ...\n", packet->msgID);

    bool confirmation_received = false;
    int attempts = 0;
    const int max_attempts = 5;

    // Start the timer
    if (xTimerStart(xTimer, 0) != pdPASS)
    {
        // Timer start failed
        printf("Failed to start timer.\n");
        return LORA_FAILED_SEND_PACKET;
    }

    while (!confirmation_received && attempts < max_attempts)
    {
        printf("Attempt %d: Waiting for confirmation...\n", attempts + 1);
        status = lora_receive(&receive_packet);
        if (status == LORA_OK)
        {
            printf("Received message with ID %d\n", receive_packet.msgID);
            if (receive_packet.msgID == packet->msgID)
            {
                xTimerStop(xTimer, 0);
                printf("Received confirmation for message ID %d!\n", packet->msgID);
                return LORA_OK;
            }
        }
        else
        {
            printf("Attempt %d: No message received. Retrying...\n", attempts + 1);
        }
        attempts++;
    }

    printf("Could not receive confirmation for message ID %d after %d attempts!\n", packet->msgID, max_attempts);
    return LORA_FAILED_RECEIVE_PACKET;
}

lora_status_t lora_receive(packet_t *packet)
{
    // Buffer to hold the received packet
    uint8_t buffer[PACKET_SIZE] = {0};
    uint8_t length = 0;

    lora_receive_mode(); // Put into receive mode

    while (1)
    {
        bool hasReceived = false;
        bool crc_error = false;
        lora_received(&hasReceived, &crc_error);

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

            // @TODO Interpret the size from the dataType
            memcpy(packet->data, &buffer[META_DATA_SIZE], DATA_SIZE);
            return crc_error ? LORA_CRC_ERROR : LORA_OK;
        }
        lora_delay(20);
    }
}

lora_status_t lora_receive_confirmation(packet_t *packet)
{

    packet_t confirmation_packet;

    lora_receive(packet);

    printf("Received message ID %d\n", packet->msgID);

    confirmation_packet.version = packet->version;
    confirmation_packet.id = packet->id;
    confirmation_packet.msgID = packet->msgID;
    confirmation_packet.msgCount = 1;
    confirmation_packet.dataType = 0;

    confirmation_packet.data[0] = 0;
    confirmation_packet.data[1] = 1;
    confirmation_packet.data[2] = 2;

    printf("Sending confirmation for message ID %d!\n", confirmation_packet.msgID);
    lora_delay(2000); // Delay for 1 second (adjust as needed)
    return lora_send(&confirmation_packet);
}