#include "bq27441.h"
#include "i2c.h"
#include "esp_log.h"

static const char *TAG = "BQ27441";

// Control subcommands
#define BQ27441_CONTROL_STATUS 0x0000
#define BQ27441_DEVICE_TYPE 0x0001
#define BQ27441_FW_VERSION 0x0002
#define BQ27441_DM_CODE 0x0004
#define BQ27441_PREV_MACWRITE 0x0007
#define BQ27441_CHEM_ID 0x0008
#define BQ27441_SET_CFGUPDATE 0x0013
#define BQ27441_SOFT_RESET 0x0042
#define BQ27441_EXIT_CFGUPDATE 0x0043

// Data class IDs
#define BQ27441_ID_STATE 82

// Register addresses
#define BQ27441_CMD_SOC 0x1C
#define BQ27441_CMD_VOLTAGE 0x04

static esp_err_t bq27441_unseal(void);
static esp_err_t bq27441_enter_config_mode(void);
static esp_err_t bq27441_exit_config_mode(void);
static esp_err_t bq27441_write_extended_data(uint8_t classID, uint8_t offset, uint8_t *data, uint8_t len);
static esp_err_t bq27441_read_flags(uint16_t *flags);

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
    err = bq27441_enter_config_mode();
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to enter config mode: %s", esp_err_to_name(err));
        return err;
    }

    // Set design capacity for 18650 battery
    err = bq27441_set_design_capacity(3400); // Assuming 3400mAh for 18650 battery
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to set design capacity: %s", esp_err_to_name(err));
        return err;
    }

    // Set terminate voltage (example value, adjust as needed)
    uint16_t terminate_voltage = 3200; // 3200 mV
    uint8_t terminate_voltage_data[2] = {terminate_voltage & 0xFF, (terminate_voltage >> 8) & 0xFF};
    err = bq27441_write_extended_data(BQ27441_ID_STATE, 12, terminate_voltage_data, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to set terminate voltage: %s", esp_err_to_name(err));
        return err;
    }

    // Exit config mode
    err = bq27441_exit_config_mode();
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
    err = bq27441_enter_config_mode();
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to enter config mode: %s", esp_err_to_name(err));
        return err;
    }

    // Write design capacity to extended data
    uint8_t capacity_data[2] = {capacity & 0xFF, (capacity >> 8) & 0xFF};
    err = bq27441_write_extended_data(BQ27441_ID_STATE, 10, capacity_data, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write design capacity: %s", esp_err_to_name(err));
        return err;
    }

    // Exit configuration mode
    err = bq27441_exit_config_mode();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to exit config mode: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t bq27441_read_design_capacity(uint16_t *capacity)
{
    uint8_t data[2];
    esp_err_t err = i2c_read(BQ27441_ADDR, 0x4A, data, 2); // Reading from offset 10 in subclass 82
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
    uint8_t data[1];
    esp_err_t err = i2c_read(BQ27441_ADDR, BQ27441_CMD_SOC, data, 1); // Command to read State of Charge
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
    uint8_t data[2];
    esp_err_t err = i2c_read(BQ27441_ADDR, BQ27441_CMD_VOLTAGE, data, 2); // Command to read Voltage
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
    uint8_t unseal_key[2] = {0x14, 0x04}; // Typical unseal key for BQ27441
    esp_err_t err;

    // Send the unseal key twice
    err = i2c_write(BQ27441_ADDR, 0x00, unseal_key, 2);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to unseal device: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_write(BQ27441_ADDR, 0x00, unseal_key, 2);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to unseal device: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t bq27441_enter_config_mode(void)
{
    uint8_t cmd[2] = {0x13, 0x00}; // SET_CFGUPDATE command
    return i2c_write(BQ27441_ADDR, 0x00, cmd, 2);
}

static esp_err_t bq27441_exit_config_mode(void)
{
    uint8_t cmd[2] = {0x42, 0x00}; // SOFT_RESET command
    return i2c_write(BQ27441_ADDR, 0x00, cmd, 2);
}

static esp_err_t bq27441_write_extended_data(uint8_t classID, uint8_t offset, uint8_t *data, uint8_t len)
{
    esp_err_t err;

    // Block data control
    uint8_t block_data_control = 0x00;
    err = i2c_write(BQ27441_ADDR, 0x61, &block_data_control, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write block data control: %s", esp_err_to_name(err));
        return err;
    }

    // Data class
    err = i2c_write(BQ27441_ADDR, 0x3E, &classID, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write data class: %s", esp_err_to_name(err));
        return err;
    }

    // Data block
    uint8_t block_offset = offset / 32;
    err = i2c_write(BQ27441_ADDR, 0x3F, &block_offset, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write data block: %s", esp_err_to_name(err));
        return err;
    }

    // Write the data
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t data_offset = offset % 32 + i;
        err = i2c_write(BQ27441_ADDR, 0x40 + data_offset, &data[i], 1);
        if (ESP_OK != err)
        {
            ESP_LOGE(TAG, "Failed to write data: %s", esp_err_to_name(err));
            return err;
        }
    }

    // Compute and write checksum
    uint8_t checksum = 0;
    uint8_t data_block[32];

    err = i2c_read(BQ27441_ADDR, 0x40, data_block, 32);

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

    err = i2c_write(BQ27441_ADDR, 0x60, &checksum, 1);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to write checksum: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t bq27441_read_flags(uint16_t *flags)
{
    uint8_t data[2];
    esp_err_t err = i2c_read(BQ27441_ADDR, 0x06, data, 2); // Command to read flags

    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to read flags: %s", esp_err_to_name(err));
        return err;
    }

    *flags = (data[1] << 8) | data[0];
    return ESP_OK;
}
