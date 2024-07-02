#include "freertos/FreeRTOS.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "gauge.h"

#include "bme280_lib_support.c"

#define TAG_GAUGE "Gauge"
    

esp_err_t get_voltage(uint16_t *voltage)
{
    uint8_t data[2];
    esp_err_t status;

    // Read the high byte of the voltage from the fuel gauge
    status = i2c_read(ADDR, 0x04, &data[0], 1);
    if (status != ESP_OK) {
        ESP_LOGI(TAG_GAUGE, "Failed to read voltage from fuel gauge");
        return status;
    }

    // Read the low byte of the voltage from the fuel gauge
    status = i2c_read(ADDR, 0x05, &data[1], 1);
    if (status != ESP_OK) {
        ESP_LOGI(TAG_GAUGE, "Failed to read voltage from fuel gauge");
        return status;
    }

    // Combine the two bytes into a single 16-bit value
    *voltage = (data[0] << 8) | data[1];

    return status;
}

