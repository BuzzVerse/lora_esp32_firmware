#include "bmm150.h"
#include "bmm150_lib.h"
#include "i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG_BMM150 "BMM150"

static struct bmm150_dev dev;


esp_err_t bmm150_init_driver(uint8_t dev_addr){
    dev.intf = BMM150_I2C_INTF;
    dev.intf_ptr = &dev_addr;
    dev.read = i2c_read;
    dev.write = i2c_write;
    dev.delay_us = delay_ms;
    esp_err_t ret = bmm150_init(&dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_BMM150, "BMM150 init failed.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG_BMM150, "BMM150 init success");
    return ESP_OK;
}