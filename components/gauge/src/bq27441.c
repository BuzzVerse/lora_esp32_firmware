#include "bq27441.h"
#include "i2c.h"
#include "esp_log.h"

static const char *TAG = "BQ27441";

esp_err_t bq27441_init(void)
{
    return i2c_init();
}

esp_err_t bq27441_read_design_capacity(uint16_t *capacity)
{
    uint8_t data[2];
    esp_err_t err = i2c_write(BQ27441_ADDR, 0x3E, (uint8_t[]){0x00, 0x4A}, 2); // Write BlockDataControl() and DataClass()
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write BlockDataControl or DataClass: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_read(BQ27441_ADDR, 0x40, data, 2); // Read the design capacity
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read design capacity: %s", esp_err_to_name(err));
        return err;
    }
    *capacity = (data[1] << 8) | data[0];
    return ESP_OK;
}

esp_err_t bq27441_read_soc(uint16_t *soc)
{
    uint8_t data[2];
    esp_err_t err = i2c_read(BQ27441_ADDR, 0x2C, data, 2); // Command to read State of Charge
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read state of charge: %s", esp_err_to_name(err));
        return err;
    }
    *soc = (data[1] << 8) | data[0];
    return ESP_OK;
}

esp_err_t bq27441_read_voltage(uint16_t *voltage)
{
    uint8_t data[2];
    esp_err_t err = i2c_read(BQ27441_ADDR, 0x04, data, 2); // Command to read Voltage
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read voltage: %s", esp_err_to_name(err));
        return err;
    }
    *voltage = (data[1] << 8) | data[0];
    return ESP_OK;
}
