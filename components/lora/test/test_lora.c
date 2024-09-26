#include "esp_err.h"
#include "unity.h"
#include "lora.h"

TEST_CASE("Lora Init Test", "[lora]")
{
    esp_err_t result = lora_init();
    TEST_ASSERT_EQUAL(LORA_OK, result);
}

TEST_CASE("Lora Send Test", "[lora]")
{
    uint8_t version = 1;
    uint8_t id = 1;
    uint8_t msgID = 1;
    uint8_t msgCount = 1;
    DataType dataType = 1;
    uint8_t data[DATA_SIZE];

    packet_t packet = {version, id, msgID, msgCount, dataType, data};

    esp_err_t result = lora_send_packet(&packet);
    TEST_ASSERT_EQUAL(LORA_OK, result);
}

TEST_CASE("Lora Receive Test", "[lora]")
{
    uint8_t version = 1;
    uint8_t id = 1;
    uint8_t msgID = 1;
    uint8_t msgCount = 1;
    DataType dataType = 1;
    uint8_t data[DATA_SIZE];

    packet_t packet = {version, id, msgID, msgCount, dataType, data};

    esp_err_t result = lora_receive_packet(&packet);
    TEST_ASSERT_EQUAL(LORA_OK, result);
}