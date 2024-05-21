#ifndef LORA_H
#define LORA_H

#include <stdint.h>
#include "driver/lora_driver.h"
#include "api/driver_api.h"

#define DATA_SIZE 3
#define META_DATA_SIZE 5
#define PACKET_SIZE (DATA_SIZE + META_DATA_SIZE)

/**
 * @brief Structure representing a LoRa packet.
 *
 * @details This structure holds the data for a LoRa packet, including an ID,
 * version, message ID, message count, data type, and the actual data payload.
 */
typedef struct
{
    uint8_t version;         /**< 4 bits version + 4 bits reserved for later use */
    uint8_t id;              /**< 4 bits class + 4 bits device ID */
    uint8_t msgID;           /**< 1 byte message ID */
    uint8_t msgCount;        /**< 1 byte message count (optional, can be zero if not used) */
    uint8_t dataType;        /**< 1 byte data type */
    uint8_t data[DATA_SIZE]; /**< Data payload */
} lora_packet_t;

/**
 * @brief Initialize the LoRa module.
 *
 * @details This function initializes the LoRa driver, sets the frequency, CRC,
 * coding rate, bandwidth, and spreading factor, and dumps the LoRa registers.
 *
 * @return lora_status_t Result of the initialization.
 */
lora_status_t lora_init(void);

/**
 * @brief Send a LoRa packet.
 *
 * @details This function packs the ID, version, message ID, message count,
 * and data type into a buffer, copies the data into the buffer, and sends
 * the packet using the LoRa driver.
 *
 * @param packet Pointer to the LoRa packet to be sent.
 */
lora_status_t lora_send(lora_packet_t *packet);

/**
 * @brief Receive a LoRa packet.
 *
 * @details This function puts the LoRa module into receive mode, waits for
 * a packet to be received, and unpacks the ID, version, message ID, message
 * count, and data type from the received packet.
 *
 * @param packet Pointer to the LoRa packet structure to store the received data.
 */
lora_status_t lora_receive(lora_packet_t *packet);

lora_status_t lora_send_confirmation(lora_packet_t *packet);
lora_status_t lora_receive_confirmation(lora_packet_t *packet);

#endif // LORA_H