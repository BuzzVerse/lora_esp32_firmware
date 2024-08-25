#include "bmm150.h"
#include "bmm150_lib.h"
#include "i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG_BMM150 "BMM150"

static struct bmm150_dev dev;

esp_err_t bmm150_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *((uint8_t *)intf_ptr); // Assuming intf_ptr holds the device address
    return i2c_read(device_addr, reg_addr, data, length);
}

esp_err_t bmm150_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *((uint8_t *)intf_ptr);
    return i2c_write(device_addr, reg_addr, (uint8_t *)data, length);
}

void bmm150_delay_us(uint32_t period, void *intf_ptr)
{
    delay_ms(period / 1000); // Convert microseconds to milliseconds
}

esp_err_t bmm150_init_driver(uint8_t dev_addr)
{
    dev.intf = BMM150_I2C_INTF;
    dev.intf_ptr = &dev_addr;
    dev.read = bmm150_i2c_read;
    dev.write = bmm150_i2c_write;
    dev.delay_us = bmm150_delay_us;


    esp_err_t ret = bmm150_init(&dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_BMM150, "BMM150 init failed.");
        return ESP_FAIL;
    }
    uint8_t i, status = 0;
    bmm150_soft_reset(&dev);

    // Verify chip ID
    if (dev.chip_id != BMM150_CHIP_ID)
    {
        ESP_LOGE(TAG_BMM150, "Unexpected chip ID: 0x%02X", dev.chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG_BMM150, "BMM150 init success");
    
    return ESP_OK;
}

esp_err_t set_config(struct bmm150_settings *settings)
{
    esp_err_t rslt;

#if 1// without this sensor does not work
    uint8_t dev_addr = 0x13;
    dev.intf_ptr = &dev_addr;
#endif
    // Set power mode to normal
    settings->pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(settings, &dev);
    if (rslt != BMM150_OK)
    {
        ESP_LOGE(TAG_BMM150, "Error setting power mode: %d", rslt);
        return ESP_FAIL;
    }

    // Set the preset mode for sensor
    settings->preset_mode = BMM150_PRESETMODE_REGULAR; // Example preset mode
    rslt = bmm150_set_presetmode(settings, &dev);
    if (rslt != BMM150_OK)
    {
        ESP_LOGE(TAG_BMM150, "Error setting preset mode: %d", rslt);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bmm150_read_mag_data_driver(struct bmm150_mag_data *mag_data)
{
  #if 1// without this sensor does not work
  uint8_t dev_addr = 0x13;
  dev.intf_ptr = &dev_addr;
  #endif

    int8_t rslt = bmm150_read_mag_data(mag_data, &dev);

    if (rslt != BMM150_OK)
    {
        ESP_LOGE(TAG_BMM150, "Error reading magnetometer data: %d", rslt);
        return ESP_FAIL;
    }
    return ESP_OK;
    bmm150_debug();
}
#if 1
void bmm150_dump_registers(uint8_t dev_addr)
{
  uint8_t i, status;


  dev.intf_ptr = &dev_addr;
  //  uint8_t device_addr = *((uint8_t *)intf_ptr); // Assuming intf_ptr holds the device address
  ESP_LOGI(TAG_BMM150,  "==============REGISTER DUMP===================");
  for (i = 0x40; i <= 0x71; i++) {
    bmm150_i2c_read(i,&status,1,dev.intf_ptr);
    ESP_LOGI(TAG_BMM150,"|| addr:0x%2x || val:0x%2x ", i, status);
  }
  ESP_LOGI(TAG_BMM150,  "==============================================");
}
#endif

void bmm150_debug(void)
{
  ESP_LOGI(TAG_BMM150, "dbg:0x%x", *((char *)dev.intf_ptr));
}
