#include "bme280.h"
#include "bme280_lib.h"
#include "esp_log.h"
#include "i2c.h"
#include <string.h>

#define TAG_BME280 "BME280"

struct bme280_t bme280; // This is the BME280-specific data
static bme280_config_t bme280_config;

void bme280_delay_msec(u32 ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

int8_t bme280_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
    if (NULL == reg_data || 0 == len)
    {
        return -1; // Return -1 on error
    }

    esp_err_t err = i2c_write(dev_addr, reg_addr, reg_data, len);
    return (ESP_OK == err) ? 0 : -1; // Return 0 on success, -1 on error
}

int8_t bme280_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
    if (NULL == reg_data || 0 == len)
    {
        return -1; // Return -1 on error
    }

    esp_err_t err = i2c_read(dev_addr, reg_addr, reg_data, len);
    return (ESP_OK == err) ? 0 : -1; // Return 0 on success, -1 on error
}

esp_err_t bme280_init_driver(sensor_context_t *sensor_config)
{
    if (NULL == sensor_config)
    {
        ESP_LOGE(TAG_BME280, "Sensor config is NULL.");
        return ESP_ERR_INVALID_ARG;
    }

    if (NULL == sensor_config->driver_data)
    {
        ESP_LOGE(TAG_BME280, "Sensor driver data is NULL.");
        return ESP_ERR_INVALID_ARG;
    }

    bme280_config = *((bme280_config_t *)sensor_config->driver_data);

    // Cast driver_data to the I2C address correctly
    bme280.dev_addr = bme280_config.i2c_address;
    bme280.delay_msec = bme280_delay_msec;
    bme280.bus_write = bme280_i2c_write;
    bme280.bus_read = bme280_i2c_read;

    // Print the dev_addr to confirm the address being used
    ESP_LOGD(TAG_BME280, "BME280 I2C address: 0x%02X", bme280.dev_addr);

    if (SUCCESS != bme280_init(&bme280))
    {
        ESP_LOGE(TAG_BME280, "BME280 init failed.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_BME280, "BME280 init success");
    return ESP_OK;
}

esp_err_t bme280_set_oversamp(const bme280_oversampling_t oversamp_pressure, const bme280_oversampling_t oversamp_temperature, const bme280_oversampling_t oversamp_humidity)
{
    if (SUCCESS != bme280_set_oversamp_pressure(oversamp_pressure) ||
        SUCCESS != bme280_set_oversamp_temperature(oversamp_temperature) ||
        SUCCESS != bme280_set_oversamp_humidity(oversamp_humidity))
    {
        ESP_LOGE(TAG_BME280, "BME280 set oversamp failed.");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t bme280_set_settings(const bme280_standby_time_t standby_time, const bme280_filter_coeff_t filter_coeff, const bme280_power_mode_t power_mode)
{
    if (SUCCESS != bme280_set_standby_durn(standby_time) ||
        SUCCESS != bme280_set_filter(filter_coeff) ||
        SUCCESS != bme280_set_power_mode(power_mode))
    {
        ESP_LOGE(TAG_BME280, "BME280 set settings failed.");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t bme280_read_pressure(double *pressure)
{
    if (NULL == pressure)
    {
        ESP_LOGE(TAG_BME280, "Pressure pointer is NULL.");
        return ESP_ERR_INVALID_ARG;
    }

    signed int uncomp_pressure;

    if (SUCCESS == bme280_read_uncomp_pressure(&uncomp_pressure))
    {
        *pressure = bme280_compensate_pressure_double(uncomp_pressure);
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 uncomp pressure read failed.");
        return ESP_FAIL;
    }
}

esp_err_t bme280_read_temperature(double *temperature)
{
    if (NULL == temperature)
    {
        ESP_LOGE(TAG_BME280, "Temperature pointer is NULL.");
        return ESP_ERR_INVALID_ARG;
    }

    signed int uncomp_temperature;

    if (SUCCESS == bme280_read_uncomp_temperature(&uncomp_temperature))
    {
        *temperature = bme280_compensate_temperature_double(uncomp_temperature);
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 uncomp temperature read failed.");
        return ESP_FAIL;
    }
}

esp_err_t bme280_read_humidity(double *humidity)
{
    if (NULL == humidity)
    {
        ESP_LOGE(TAG_BME280, "Humidity pointer is NULL.");
        return ESP_ERR_INVALID_ARG;
    }

    signed int uncomp_humidity;

    if (SUCCESS == bme280_read_uncomp_humidity(&uncomp_humidity))
    {
        *humidity = bme280_compensate_humidity_double(uncomp_humidity);
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 uncomp humidity read failed.");
        return ESP_FAIL;
    }
}

// Implement the sensor interface functions
esp_err_t bme280_sensor_init(void *context)
{
    if (NULL == context)
    {
        ESP_LOGE(TAG_BME280, "Sensor context is NULL.");
        return ESP_ERR_INVALID_ARG;
    }

    sensor_context_t *ctx = (sensor_context_t *)context;
    return bme280_init_driver(ctx);
}

int bme280_sensor_read(void *context, uint8_t *data, const size_t length)
{
    if (NULL == data)
    {
        ESP_LOGE(TAG_BME280, "Data buffer is NULL.");
        return ESP_FAIL;
    }

    if (BME280_DATA_SIZE > length)
    {
        return ESP_FAIL; // Insufficient buffer size
    }

    double temperature, pressure, humidity;

    if (ESP_OK != bme280_read_temperature(&temperature) ||
        ESP_OK != bme280_read_pressure(&pressure) ||
        ESP_OK != bme280_read_humidity(&humidity))
    {
        return ESP_FAIL;
    }

    // Store the data in the provided buffer
    memcpy(data, &temperature, sizeof(double));
    memcpy(data + sizeof(double), &pressure, sizeof(double));
    memcpy(data + 2 * sizeof(double), &humidity, sizeof(double));

    return ESP_OK; // Success
}

int bme280_sensor_write(void *context, const uint8_t *data, const size_t length)
{
    // No specific write operations are necessary for BME280 in this context
    return ESP_OK; // Success
}

// Define the sensor interface for BME280 AFTER all functions are defined
sensor_interface_t bme280_interface = {
    .init = bme280_sensor_init,
    .read = bme280_sensor_read,
    .write = bme280_sensor_write,
};
