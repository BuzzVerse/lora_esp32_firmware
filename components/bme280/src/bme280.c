#include "bme280.h"
#include "bme280_lib.h"
#include "i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG_BME280 "BME280"

static struct bme280_t bme280;

void delay_ms(uint32_t ticks)
{
    vTaskDelay(ticks / portTICK_PERIOD_MS);
}

esp_err_t bme280_init_driver(uint8_t dev_addr)
{
    bme280.dev_addr = dev_addr;
    bme280.delay_msec = delay_ms;
    bme280.bus_write = i2c_write;
    bme280.bus_read = i2c_read;

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
}

esp_err_t bme280_set_settings(bme280_standby_time_t standby_time, bme280_filter_coeff_t filter_coeff, bme280_power_mode_t power_mode)
{
    s32 com_rslt;

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
}

esp_err_t bme280_read_pressure(double *pressure)
{
    s32 com_rslt;
    s32 v_uncomp_pressure_s32;

    com_rslt = bme280_read_uncomp_pressure(&v_uncomp_pressure_s32);

    if (com_rslt == SUCCESS)
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

    com_rslt = bme280_read_uncomp_temperature(&v_uncomp_temperature_s32);

    if (com_rslt == SUCCESS)
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

    com_rslt = bme280_read_uncomp_humidity(&v_uncomp_humidity_s32);

    if (com_rslt == SUCCESS)
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
