#include "freertos/FreeRTOS.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "gauge.h"

#define TAG_GAUGE "Gauge"

// Function to read multiple bytes from the BQ27441
esp_err_t bq27441I2cReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count) {
    esp_err_t esp_rc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Write device address and register address for reading
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (GAUGE_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, subAddress, true);

    // Start a new I2C communication for reading
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (GAUGE_ADDR << 1) | I2C_MASTER_READ, true);

    // Read data from the device
    if (count > 1) {
        i2c_master_read(cmd, dest, count - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, dest + count - 1, I2C_MASTER_NACK);

    // Stop the I2C communication
    i2c_master_stop(cmd);

    // Execute the I2C command sequence
    esp_rc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);

    // Delete the I2C command link to free up resources
    i2c_cmd_link_delete(cmd);

    // Check if the read was successful
    if (esp_rc != ESP_OK) {
        ESP_LOGE(TAG_GAUGE, "I2C read failed. code: %d", esp_rc);
    }
    return esp_rc;
}

// Function to get the voltage from the BQ27441
esp_err_t get_voltage(uint16_t *voltage) {
    uint8_t data[2]; 
    esp_err_t status;

    status = bq27441I2cReadBytes(0x02, data, 2);
    if (status != ESP_OK) {
        ESP_LOGI(TAG_GAUGE, "Failed to read voltage from fuel gauge");
        return status;
    }

    *voltage = (data[0] << 8) | data[1];
    return status;
}
