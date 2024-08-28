#include "bme280.h"
#include "bme280_lib.h"
#include "i2c.h"
#include "i2c_defs.h"
#include "esp_log.h"

#define TAG_BME280 "BME280"

static struct bme280_t bme280;
static sensor_config_t *static_sensor_config = NULL; // Static variable to hold the sensor configuration

void delay_ms(u32 ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

// Wrapper for I2C write
s8 bme280_i2c_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len)
{
    esp_err_t err = i2c_write(dev_addr, reg_addr, reg_data, len);
    return (err == ESP_OK) ? 0 : -1; // Return 0 on success, -1 on error
}

// Wrapper for I2C read
s8 bme280_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len)
{
    esp_err_t err = i2c_read(dev_addr, reg_addr, reg_data, len);
    return (err == ESP_OK) ? 0 : -1; // Return 0 on success, -1 on error
}

esp_err_t bme280_init_driver(sensor_config_t *sensor_config)
{
    if (NULL == sensor_config)
    {
        ESP_LOGE(TAG_BME280, "Sensor config is NULL.");
        return ESP_FAIL;
    }

    static_sensor_config = sensor_config; // Store the config in the static variable

    bme280.dev_addr = static_sensor_config->i2c_address;
    bme280.delay_msec = delay_ms;
    bme280.bus_write = bme280_i2c_write;
    bme280.bus_read = bme280_i2c_read;

    if (SUCCESS != bme280_init(&bme280))
    {
        ESP_LOGE(TAG_BME280, "BME280 init failed.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_BME280, "BME280 init success");
    return ESP_OK;
}

esp_err_t bme280_set_oversamp(bme280_oversampling_t oversamp_pressure, bme280_oversampling_t oversamp_temperature, bme280_oversampling_t oversamp_humidity)
{
    s32 com_rslt;

    if (NULL == static_sensor_config)
    {
        ESP_LOGE(TAG_BME280, "Sensor config not initialized.");
        return ESP_FAIL;
    }

    com_rslt = bme280_set_oversamp_pressure(oversamp_pressure);
    com_rslt += bme280_set_oversamp_temperature(oversamp_temperature);
    com_rslt += bme280_set_oversamp_humidity(oversamp_humidity);

    if (SUCCESS == com_rslt)
    {
        ESP_LOGI(TAG_BME280, "BME280 set oversamp success");
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 set oversamp failed. code: %d", com_rslt);
        return ESP_FAIL;
    }
}

esp_err_t bme280_set_settings(bme280_standby_time_t standby_time, bme280_filter_coeff_t filter_coeff, bme280_power_mode_t power_mode)
{
    s32 com_rslt;

    if (NULL == static_sensor_config)
    {
        ESP_LOGE(TAG_BME280, "Sensor config not initialized.");
        return ESP_FAIL;
    }

    com_rslt = bme280_set_standby_durn(standby_time);
    com_rslt += bme280_set_filter(filter_coeff);
    com_rslt += bme280_set_power_mode(power_mode);

    if (SUCCESS == com_rslt)
    {
        ESP_LOGI(TAG_BME280, "BME280 set settings success");
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 set settings failed. code: %d", com_rslt);
        return ESP_FAIL;
    }
}

esp_err_t bme280_read_pressure(double *pressure)
{
    s32 com_rslt;
    s32 v_uncomp_pressure_s32;

    if (NULL == static_sensor_config)
    {
        ESP_LOGE(TAG_BME280, "Sensor config not initialized.");
        return ESP_FAIL;
    }

    com_rslt = bme280_read_uncomp_pressure(&v_uncomp_pressure_s32);

    if (SUCCESS == com_rslt)
    {
        *pressure = bme280_compensate_pressure_double(v_uncomp_pressure_s32);
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 uncomp pressure read failed. code: %d", com_rslt);
        return ESP_FAIL;
    }
}

esp_err_t bme280_read_temperature(double *temperature)
{
    s32 com_rslt;
    s32 v_uncomp_temperature_s32;

    if (NULL == static_sensor_config)
    {
        ESP_LOGE(TAG_BME280, "Sensor config not initialized.");
        return ESP_FAIL;
    }

    com_rslt = bme280_read_uncomp_temperature(&v_uncomp_temperature_s32);

    if (SUCCESS == com_rslt)
    {
        *temperature = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 uncomp temperature read failed. code: %d", com_rslt);
        return ESP_FAIL;
    }
}

esp_err_t bme280_read_humidity(double *humidity)
{
    s32 com_rslt;
    s32 v_uncomp_humidity_s32;

    if (NULL == static_sensor_config)
    {
        ESP_LOGE(TAG_BME280, "Sensor config not initialized.");
        return ESP_FAIL;
    }

    com_rslt = bme280_read_uncomp_humidity(&v_uncomp_humidity_s32);

    if (SUCCESS == com_rslt)
    {
        *humidity = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 uncomp humidity read failed. code: %d", com_rslt);
        return ESP_FAIL;
    }
}
