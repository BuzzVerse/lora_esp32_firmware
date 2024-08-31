#include "bq27441.h"
#include "bq27441_defs.h"
#include "i2c.h"
#include "esp_log.h"
#include <stdbool.h>
#include <string.h>

static const char *TAG = "BQ27441";

// Function declarations
static esp_err_t bq27441_unseal(const bq27441_config_t *config);
static esp_err_t bq27441_toggle_config_mode(const bq27441_config_t *config, bool enter);
static esp_err_t bq27441_write_extended_data(const bq27441_config_t *config, uint8_t classID, uint8_t offset, uint8_t *data, uint8_t len);

esp_err_t bq27441_init(sensor_context_t *ctx)
{
    if (NULL == ctx || NULL == ctx->driver_data)
    {
        ESP_LOGE(TAG, "Invalid context or driver data");
        return ESP_ERR_INVALID_ARG;
    }

    bq27441_config_t *config = (bq27441_config_t *)ctx->driver_data;
    esp_err_t err = ESP_OK;

    // print the config
    ESP_LOGD(TAG, "BQ27441 Config: I2C Address: 0x%02X, Design Capacity: %d mAh, Terminate Voltage: %d mV", config->i2c_address, config->design_capacity, config->terminate_voltage);

    // Ensure the device is unsealed
    err = bq27441_unseal(config);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to unseal device: %s", esp_err_to_name(err));
        return err;
    }

    // Enter config mode
    err = bq27441_toggle_config_mode(config, true);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to enter config mode: %s", esp_err_to_name(err));
        return err;
    }

    // Set design capacity
    err = bq27441_set_design_capacity(ctx, config->design_capacity);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to set design capacity: %s", esp_err_to_name(err));
        return err;
    }

    // Set terminate voltage
    uint16_t terminate_voltage = config->terminate_voltage;
    uint8_t terminate_voltage_data[2] = {terminate_voltage & 0xFF, (terminate_voltage >> 8) & 0xFF};
    err = bq27441_write_extended_data(config, BQ27441_ID_STATE, BQ27441_TERMINATE_VOLTAGE_OFFSET, terminate_voltage_data, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to set terminate voltage: %s", esp_err_to_name(err));
        return err;
    }

    // Exit config mode
    err = bq27441_toggle_config_mode(config, false);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to exit config mode: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t bq27441_set_design_capacity(sensor_context_t *ctx, uint16_t capacity)
{
    if (NULL == ctx || NULL == ctx->driver_data)
    {
        ESP_LOGE(TAG, "Invalid context or driver data");
        return ESP_ERR_INVALID_ARG;
    }

    bq27441_config_t *config = (bq27441_config_t *)ctx->driver_data;
    esp_err_t err;

    // Unseal the device
    err = bq27441_unseal(config);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to unseal device: %s", esp_err_to_name(err));
        return err;
    }

    // Enter configuration mode
    err = bq27441_toggle_config_mode(config, true);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to enter config mode: %s", esp_err_to_name(err));
        return err;
    }

    // Write design capacity to extended data
    uint8_t capacity_data[2] = {capacity & 0xFF, (capacity >> 8) & 0xFF};
    err = bq27441_write_extended_data(config, BQ27441_ID_STATE, BQ27441_DESIGN_CAPACITY_OFFSET, capacity_data, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write design capacity: %s", esp_err_to_name(err));
        return err;
    }

    // Exit configuration mode
    err = bq27441_toggle_config_mode(config, false);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to exit config mode: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t bq27441_read_design_capacity(sensor_context_t *ctx, uint16_t *capacity)
{
    if (NULL == ctx || NULL == ctx->driver_data || NULL == capacity)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bq27441_config_t *config = (bq27441_config_t *)ctx->driver_data;
    uint8_t data[2];
    esp_err_t err = i2c_read(config->i2c_address, BQ27441_CMD_READ_DESIGN_CAPACITY, data, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to read design capacity: %s", esp_err_to_name(err));
        return err;
    }
    *capacity = (data[1] << 8) | data[0];
    return ESP_OK;
}

esp_err_t bq27441_read_soc(sensor_context_t *ctx, uint8_t *soc)
{
    if (NULL == ctx || NULL == ctx->driver_data || NULL == soc)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bq27441_config_t *config = (bq27441_config_t *)ctx->driver_data;
    uint8_t data[1];
    esp_err_t err = i2c_read(config->i2c_address, BQ27441_CMD_SOC, data, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to read state of charge: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGD(TAG, "Raw SoC data: 0x%02x", data[0]);
    *soc = data[0];
    return ESP_OK;
}

esp_err_t bq27441_read_voltage(sensor_context_t *ctx, uint16_t *voltage)
{
    if (NULL == ctx || NULL == ctx->driver_data || NULL == voltage)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bq27441_config_t *config = (bq27441_config_t *)ctx->driver_data;
    uint8_t data[2];
    esp_err_t err = i2c_read(config->i2c_address, BQ27441_CMD_VOLTAGE, data, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to read voltage: %s", esp_err_to_name(err));
        return err;
    }
    *voltage = (data[1] << 8) | data[0];
    return ESP_OK;
}

static esp_err_t bq27441_unseal(const bq27441_config_t *config)
{
    uint8_t unseal_key[2] = {BQ27441_UNSEAL_KEY0, BQ27441_UNSEAL_KEY1}; // Use defined unseal keys
    esp_err_t err;

    // Send the unseal key twice
    err = i2c_write(config->i2c_address, BQ27441_CMD_CONTROL, unseal_key, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to unseal device: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_write(config->i2c_address, BQ27441_CMD_CONTROL, unseal_key, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to unseal device: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t bq27441_toggle_config_mode(const bq27441_config_t *config, bool enter)
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

    return i2c_write(config->i2c_address, BQ27441_CMD_CONTROL, cmd, 2);
}

static esp_err_t bq27441_write_extended_data(const bq27441_config_t *config, uint8_t classID, uint8_t offset, uint8_t *data, uint8_t len)
{
    esp_err_t err;

    // Block data control
    uint8_t block_data_control = 0x00;
    err = i2c_write(config->i2c_address, BQ27441_REG_BLOCK_DATA_CONTROL, &block_data_control, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write block data control: %s", esp_err_to_name(err));
        return err;
    }

    // Data class
    err = i2c_write(config->i2c_address, BQ27441_REG_CLASS_ID, &classID, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write data class: %s", esp_err_to_name(err));
        return err;
    }

    // Data block
    uint8_t block_offset = offset / 32;
    err = i2c_write(config->i2c_address, BQ27441_REG_BLOCK_OFFSET, &block_offset, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write data block: %s", esp_err_to_name(err));
        return err;
    }

    // Write the data
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t data_offset = offset % 32 + i;
        err = i2c_write(config->i2c_address, BQ27441_REG_BLOCK_DATA + data_offset, &data[i], 1);
        if (ESP_OK != err)
        {
            ESP_LOGE(TAG, "Failed to write data: %s", esp_err_to_name(err));
            return err;
        }
    }

    // Compute and write checksum
    uint8_t checksum = 0;
    uint8_t data_block[32];

    err = i2c_read(config->i2c_address, BQ27441_REG_BLOCK_DATA, data_block, 32);
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

    err = i2c_write(config->i2c_address, BQ27441_REG_CHECKSUM, &checksum, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write checksum: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

int bq27441_sensor_init(void *context)
{
    return (ESP_OK == bq27441_init((sensor_context_t *)context)) ? 0 : -1;
}

int bq27441_sensor_read(void *context, uint8_t *data, size_t length)
{
    if (3 * sizeof(uint16_t) > length)
    {
        return -1; // Insufficient buffer size
    }

    sensor_context_t *ctx = (sensor_context_t *)context;
    uint16_t capacity, voltage;
    uint8_t soc;

    if (ESP_OK != bq27441_read_design_capacity(ctx, &capacity) ||
        ESP_OK != bq27441_read_soc(ctx, &soc) ||
        ESP_OK != bq27441_read_voltage(ctx, &voltage))
    {
        return -1; // Error occurred while reading
    }

    // Store the data in the provided buffer
    memcpy(data, &capacity, sizeof(uint16_t));
    memcpy(data + sizeof(uint16_t), &soc, sizeof(uint8_t));
    memcpy(data + sizeof(uint16_t) + sizeof(uint8_t), &voltage, sizeof(uint16_t));

    return 0; // Success
}

int bq27441_sensor_write(void *context, const uint8_t *data, size_t length)
{
    // BQ27441 may not have significant write operations in this simplified case
    return 0; // Success
}

sensor_interface_t bq27441_interface = {
    .init = bq27441_sensor_init,
    .read = bq27441_sensor_read,
    .write = bq27441_sensor_write,
};
