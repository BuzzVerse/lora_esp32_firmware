#include "bq27441.h"
#include "bq27441_defs.h"
#include "i2c.h"
#include "esp_log.h"
#include <stdbool.h>

static const char *TAG = "BQ27441";

// Function declarations
static esp_err_t bq27441_unseal(void);
static esp_err_t bq27441_toggle_config_mode(bool enter);
static esp_err_t bq27441_write_extended_data(uint8_t classID, uint8_t offset, uint8_t *data, uint8_t len);

esp_err_t bq27441_init(void)
{
    esp_err_t err = ESP_OK;

    // Ensure the device is unsealed
    err = bq27441_unseal();
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to unseal device: %s", esp_err_to_name(err));
        return err;
    }

    // Enter config mode
    err = bq27441_toggle_config_mode(true);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to enter config mode: %s", esp_err_to_name(err));
        return err;
    }

    // Set design capacity
    err = bq27441_set_design_capacity(CONFIG_BQ27441_DESIGN_CAPACITY); // 3400mAh for 18650 battery
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to set design capacity: %s", esp_err_to_name(err));
        return err;
    }

    // Set terminate voltage
    uint16_t terminate_voltage = CONFIG_BQ27441_TERMINATE_VOLTAGE; // 3200 mV for 18650 battery
    uint8_t terminate_voltage_data[2] = {terminate_voltage & 0xFF, (terminate_voltage >> 8) & 0xFF};
    err = bq27441_write_extended_data(BQ27441_ID_STATE, BQ27441_TERMINATE_VOLTAGE_OFFSET, terminate_voltage_data, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to set terminate voltage: %s", esp_err_to_name(err));
        return err;
    }

    // Exit config mode
    err = bq27441_toggle_config_mode(false);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to exit config mode: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t bq27441_set_design_capacity(uint16_t capacity)
{
    esp_err_t err;

    // Unseal the device
    err = bq27441_unseal();
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to unseal device: %s", esp_err_to_name(err));
        return err;
    }

    // Enter configuration mode
    err = bq27441_toggle_config_mode(true);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to enter config mode: %s", esp_err_to_name(err));
        return err;
    }

    // Write design capacity to extended data
    uint8_t capacity_data[2] = {capacity & 0xFF, (capacity >> 8) & 0xFF};
    err = bq27441_write_extended_data(BQ27441_ID_STATE, BQ27441_DESIGN_CAPACITY_OFFSET, capacity_data, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write design capacity: %s", esp_err_to_name(err));
        return err;
    }

    // Exit configuration mode
    err = bq27441_toggle_config_mode(false);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to exit config mode: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t bq27441_read_design_capacity(uint16_t *capacity)
{
    if (capacity == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2];
    esp_err_t err = i2c_read(BQ27441_ADDR, BQ27441_CMD_READ_DESIGN_CAPACITY, data, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to read design capacity: %s", esp_err_to_name(err));
        return err;
    }
    *capacity = (data[1] << 8) | data[0];
    return ESP_OK;
}

esp_err_t bq27441_read_soc(uint8_t *soc)
{
    if (soc == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[1];
    esp_err_t err = i2c_read(BQ27441_ADDR, BQ27441_CMD_SOC, data, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to read state of charge: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGD(TAG, "Raw SoC data: 0x%02x", data[0]);
    *soc = data[0];
    return ESP_OK;
}

esp_err_t bq27441_read_voltage(uint16_t *voltage)
{
    if (voltage == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2];
    esp_err_t err = i2c_read(BQ27441_ADDR, BQ27441_CMD_VOLTAGE, data, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to read voltage: %s", esp_err_to_name(err));
        return err;
    }
    *voltage = (data[1] << 8) | data[0];
    return ESP_OK;
}

static esp_err_t bq27441_unseal(void)
{
    uint8_t unseal_key[2] = {BQ27441_UNSEAL_KEY0, BQ27441_UNSEAL_KEY1}; // Use defined unseal keys
    esp_err_t err;

    // Send the unseal key twice
    err = i2c_write(BQ27441_ADDR, BQ27441_CMD_CONTROL, unseal_key, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to unseal device: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_write(BQ27441_ADDR, BQ27441_CMD_CONTROL, unseal_key, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to unseal device: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t bq27441_toggle_config_mode(bool enter)
{
    uint8_t cmd[2];
    if (enter)
    {
        cmd[0] = BQ27441_CMD_SET_CFGUPDATE & 0xFF;
        cmd[1] = (BQ27441_CMD_SET_CFGUPDATE >> 8) & 0xFF;
    }
    else
    {
        cmd[0] = BQ27441_CMD_SOFT_RESET & 0xFF;
        cmd[1] = (BQ27441_CMD_SOFT_RESET >> 8) & 0xFF;
    }

    return i2c_write(BQ27441_ADDR, BQ27441_CMD_CONTROL, cmd, 2);
}

static esp_err_t bq27441_write_extended_data(uint8_t classID, uint8_t offset, uint8_t *data, uint8_t len)
{
    esp_err_t err;

    // Block data control
    uint8_t block_data_control = 0x00;
    err = i2c_write(BQ27441_ADDR, BQ27441_REG_BLOCK_DATA_CONTROL, &block_data_control, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write block data control: %s", esp_err_to_name(err));
        return err;
    }

    // Data class
    err = i2c_write(BQ27441_ADDR, BQ27441_REG_CLASS_ID, &classID, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write data class: %s", esp_err_to_name(err));
        return err;
    }

    // Data block
    uint8_t block_offset = offset / 32;
    err = i2c_write(BQ27441_ADDR, BQ27441_REG_BLOCK_OFFSET, &block_offset, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write data block: %s", esp_err_to_name(err));
        return err;
    }

    // Write the data
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t data_offset = offset % 32 + i;
        err = i2c_write(BQ27441_ADDR, BQ27441_REG_BLOCK_DATA + data_offset, &data[i], 1);
        if (ESP_OK != err)
        {
            ESP_LOGE(TAG, "Failed to write data: %s", esp_err_to_name(err));
            return err;
        }
    }

    // Compute and write checksum
    uint8_t checksum = 0;
    uint8_t data_block[32];

    err = i2c_read(BQ27441_ADDR, BQ27441_REG_BLOCK_DATA, data_block, 32);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to read data block: %s", esp_err_to_name(err));
        return err;
    }

    for (uint8_t i = 0; i < 32; i++)
    {
        checksum += data_block[i];
    }
    checksum = 255 - checksum;

    err = i2c_write(BQ27441_ADDR, BQ27441_REG_CHECKSUM, &checksum, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write checksum: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}
