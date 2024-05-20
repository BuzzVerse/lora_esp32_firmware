// lora.h

#ifndef LORA_H
#define LORA_H

#include <stdint.h>

#define DATA_SIZE 55  // Adjusted size to fit within 64 bytes total

typedef struct {
    uint8_t id;             // 4 bits class + 4 bits device id
    uint8_t version;        // 4 bits version + 4 bits reserved for later use
    uint8_t msgID;          // 1 byte message id
    uint8_t msgCount;       // 1 byte message count (optional, can be zero if not used)
    uint8_t dataType;       // 1 byte data type
    uint8_t data[DATA_SIZE];// Up to 55 bytes of data
} LoRaPacket;

// Function prototypes
void lora_init(void);
void lora_send(const LoRaPacket *packet);
void lora_receive(LoRaPacket *packet);

#endif // LORA_H
