#include "i2c.h"
#include "i2c_defs.h"
#include "sensor.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "I2C_MASTER_CUSTOM";

static i2c_master_bus_handle_t i2c_bus_handle = NULL; // Handle for the I2C master bus

esp_err_t i2c_init(void)
{
    // Configure the I2C master bus
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT, // You can choose another clock source if needed
        .glitch_ignore_cnt = 0,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t i2c_device_init(i2c_device_config_t *dev_cfg, i2c_master_dev_handle_t *dev_handle)
{
    esp_err_t err = i2c_master_bus_add_device(i2c_bus_handle, dev_cfg, dev_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C device: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t i2c_write_custom(sensor_interface_t *intf, uint8_t reg_addr, uint8_t *data, size_t data_len)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf->config.i2c.dev_handle;

    uint8_t cmd_data[data_len + 1];
    cmd_data[0] = reg_addr;
    memcpy(&cmd_data[1], data, data_len);

    esp_err_t err = i2c_master_transmit(dev_handle, cmd_data, data_len + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t i2c_read_custom(sensor_interface_t *intf, uint8_t reg_addr, uint8_t *data, size_t data_len)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf->config.i2c.dev_handle;

    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, data_len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
    }

    return err;
}
