#include "esp_err.h"
#include "unity.h"
#include "i2c.h"

TEST_CASE("I2C Init Test", "[i2c]")
{
    esp_err_t result = i2c_init();
    TEST_ASSERT_EQUAL(ESP_OK, result);
}

TEST_CASE("I2C Write Test", "[i2c]")
{
    uint8_t dev_addr = 0x50;
    uint8_t reg_addr = 0x20;
    uint8_t data[2] = {0xAB, 0xCD};
    esp_err_t result = i2c_write(dev_addr, reg_addr, data, sizeof(data));
    TEST_ASSERT_EQUAL(ESP_OK, result);
}
