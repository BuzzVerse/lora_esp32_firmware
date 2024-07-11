#include "freertos/FreeRTOS.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "gauge.h"

#define TAG_GAUGE "Gauge"

#include "bme280.h"

esp_err_t get_voltage(uint16_t *voltage)
{
    uint8_t data[2];
    esp_err_t status;

    // Read the high byte of the voltage from the fuel gauge
    status = i2c_read(GAUGE_ADDR, 0x04, &data[0], 2);
    if (status != ESP_OK) {
        ESP_LOGI(TAG_GAUGE, "Failed to read voltage from fuel gauge");
        return status;
    }

    // Combine the two bytes into a single 16-bit value
    *voltage = (data[0] << 8) | data[1];

    return status;
}