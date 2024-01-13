#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "bme280_lib.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

#define TAG_BME280 "BME280"
struct bme280_t bme280;

// Function to initialize I2C
void i2c_master_init(void)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MASTER_SDA_IO,
        .scl_io_num = CONFIG_I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000};
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

// Function to read data from BME280 using I2C
int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int32_t iError = SUCCESS;

    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = 0;
    }
    else
    {
        iError = -1;
    }
    i2c_cmd_link_delete(cmd);

    return (int8_t)iError;
}

int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int32_t iError = SUCCESS;
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (cnt > 1)
    {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = 0;
    }
    else
    {
        iError = -1;
    }

    i2c_cmd_link_delete(cmd);

    return (int8_t)iError;
}

// Function to provide microsecond delay
void delay_us(uint32_t msek)
{
    vTaskDelay(msek / portTICK_PERIOD_MS);
}
// kinda stinky code
struct bme280_t bme280 = {
    .bus_write = i2c_write,
    .bus_read = i2c_read,
    .dev_addr = BME280_I2C_ADDRESS1,
    .delay_msec = delay_us};

esp_err_t bme280_init_driver(uint8_t dev_addr)
{

    bme280.dev_addr = dev_addr;

    s32 com_rslt;

    i2c_master_init();
    com_rslt = bme280_init(&bme280);

    if (com_rslt == SUCCESS)
    {
        ESP_LOGI(TAG_BME280, "BME280 init success");
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 init failed. code: %d", com_rslt);
        return ESP_FAIL;
    }

    return com_rslt;
}

esp_err_t bme280_set_oversamp(uint8_t oversamp_pressure, uint8_t oversamp_temperature, uint8_t oversamp_humidity)
{
    s32 com_rslt;

    oversamp_pressure = BME280_OVERSAMP_16X;
    oversamp_temperature = BME280_OVERSAMP_16X;
    oversamp_humidity = BME280_OVERSAMP_16X;

    com_rslt = bme280_set_oversamp_pressure(oversamp_pressure);
    com_rslt += bme280_set_oversamp_temperature(oversamp_temperature);
    com_rslt += bme280_set_oversamp_humidity(oversamp_humidity);

    if (com_rslt == SUCCESS)
    {
        ESP_LOGI(TAG_BME280, "BME280 set oversamp success");
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 set oversamp failed. code: %d", com_rslt);
        return ESP_FAIL;
    }

    return com_rslt;
}

esp_err_t bme280_set_settings(uint8_t standby_time, uint8_t filter_coeff, uint8_t power_mode)
{
    s32 com_rslt;

    standby_time = BME280_STANDBY_TIME_1_MS;
    filter_coeff = BME280_FILTER_COEFF_16;
    power_mode = BME280_NORMAL_MODE;

    com_rslt = bme280_set_standby_durn(standby_time);
    com_rslt += bme280_set_filter(filter_coeff);
    com_rslt += bme280_set_power_mode(power_mode);

    if (com_rslt == SUCCESS)
    {
        ESP_LOGI(TAG_BME280, "BME280 set settings success");
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 set settings failed. code: %d", com_rslt);
        return ESP_FAIL;
    }

    return com_rslt;
}

double bme280_read_pressure(void)
{
    s32 com_rslt;
    s32 v_uncomp_pressure_s32;

    com_rslt = bme280_read_uncomp_pressure(&v_uncomp_pressure_s32);

    if (com_rslt == SUCCESS)
    {
        return bme280_compensate_pressure_double(v_uncomp_pressure_s32);
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 uncomp pressure read failed. code: %d", com_rslt);
        return 0;
    }
}

double bme280_read_temperature(void)
{
    s32 com_rslt;
    s32 v_uncomp_temperature_s32;

    com_rslt = bme280_read_uncomp_temperature(&v_uncomp_temperature_s32);

    if (com_rslt == SUCCESS)
    {
        return bme280_compensate_temperature_double(v_uncomp_temperature_s32);
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 uncomp temperature read failed. code: %d", com_rslt);
        return 0;
    }
}

double bme280_read_humidity(void)
{
    s32 com_rslt;
    s32 v_uncomp_humidity_s32;

    com_rslt = bme280_read_uncomp_humidity(&v_uncomp_humidity_s32);

    if (com_rslt == SUCCESS)
    {
        return bme280_compensate_humidity_double(v_uncomp_humidity_s32);
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 uncomp humidity read failed. code: %d", com_rslt);
        return 0;
    }
}