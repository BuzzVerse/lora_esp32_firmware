#ifndef _LORA_H_
#define _LORA_H_

#include <stdint.h>
#include "driver/lora_driver.h"
#include "protocols/packet/packet.h"
#include "api/driver_api.h"

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
lora_status_t lora_send(packet_t *packet);

/**
 * @brief Receive a LoRa packet.
 *
 * @details This function puts the LoRa module into receive mode, waits for
 * a packet to be received, and unpacks the ID, version, message ID, message
 * count, and data type from the received packet.
 *
 * @param packet Pointer to the LoRa packet structure to store the received data.
 */
lora_status_t lora_receive(packet_t *packet);

#endif // _LORA_H_