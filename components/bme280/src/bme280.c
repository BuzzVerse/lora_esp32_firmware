#include "bme280.h"
#include "bme280_lib.h"
#include "esp_log.h"
#include "i2c.h"
#include <string.h>  // Include this for memcpy

#define TAG_BME280 "BME280"

struct bme280_t bme280; // This is the BME280-specific data
static bme280_config_t bme280_config;

// Wrapper function for the delay to match the required signature
void bme280_delay_msec(u32 ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

// Wrapper for I2C write
s8 bme280_i2c_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len) {
    esp_err_t err = i2c_write(dev_addr, reg_addr, reg_data, len);
    return (ESP_OK == err) ? 0 : -1; // Return 0 on success, -1 on error
}

// Wrapper for I2C read
s8 bme280_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len) {
    esp_err_t err = i2c_read(dev_addr, reg_addr, reg_data, len);
    return (ESP_OK == err) ? 0 : -1; // Return 0 on success, -1 on error
}

esp_err_t bme280_init_driver(sensor_context_t *sensor_config) {
    if (NULL == sensor_config) {
        ESP_LOGE(TAG_BME280, "Sensor config is NULL.");
        return ESP_FAIL;
    }

    bme280_config = *((bme280_config_t *)sensor_config->driver_data); 

    // Cast driver_data to the I2C address correctly
    bme280.dev_addr = bme280_config.i2c_address;
    bme280.delay_msec = bme280_delay_msec;
    bme280.bus_write = bme280_i2c_write;
    bme280.bus_read = bme280_i2c_read;

    // Print the dev_addr to confirm the address being used
    ESP_LOGD(TAG_BME280, "BME280 I2C address: 0x%02X", bme280.dev_addr);

    if (SUCCESS != bme280_init(&bme280)) {
        ESP_LOGE(TAG_BME280, "BME280 init failed.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_BME280, "BME280 init success");
    return ESP_OK;
}

esp_err_t bme280_set_oversamp(bme280_oversampling_t oversamp_pressure, bme280_oversampling_t oversamp_temperature, bme280_oversampling_t oversamp_humidity) {
    if (SUCCESS != bme280_set_oversamp_pressure(oversamp_pressure) ||
        SUCCESS != bme280_set_oversamp_temperature(oversamp_temperature) ||
        SUCCESS != bme280_set_oversamp_humidity(oversamp_humidity)) {
        ESP_LOGE(TAG_BME280, "BME280 set oversamp failed.");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t bme280_set_settings(bme280_standby_time_t standby_time, bme280_filter_coeff_t filter_coeff, bme280_power_mode_t power_mode) {
    if (SUCCESS != bme280_set_standby_durn(standby_time) ||
        SUCCESS != bme280_set_filter(filter_coeff) ||
        SUCCESS != bme280_set_power_mode(power_mode)) {
        ESP_LOGE(TAG_BME280, "BME280 set settings failed.");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t bme280_read_pressure(double *pressure) {
    s32 uncomp_pressure;

    if (SUCCESS == bme280_read_uncomp_pressure(&uncomp_pressure)) {
        *pressure = bme280_compensate_pressure_double(uncomp_pressure);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG_BME280, "BME280 uncomp pressure read failed.");
        return ESP_FAIL;
    }
}

esp_err_t bme280_read_temperature(double *temperature) {
    s32 uncomp_temperature;

    if (SUCCESS == bme280_read_uncomp_temperature(&uncomp_temperature)) {
        *temperature = bme280_compensate_temperature_double(uncomp_temperature);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG_BME280, "BME280 uncomp temperature read failed.");
        return ESP_FAIL;
    }
}

esp_err_t bme280_read_humidity(double *humidity) {
    s32 uncomp_humidity;

    if (SUCCESS == bme280_read_uncomp_humidity(&uncomp_humidity)) {
        *humidity = bme280_compensate_humidity_double(uncomp_humidity);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG_BME280, "BME280 uncomp humidity read failed.");
        return ESP_FAIL;
    }
}

// Implement the sensor interface functions
int bme280_sensor_init(void* context) {
    sensor_context_t* ctx = (sensor_context_t*)context;
    return (ESP_OK == bme280_init_driver(ctx)) ? 0 : -1;
}

int bme280_sensor_read(void* context, uint8_t* data, size_t length) {
    if (3 * sizeof(double) > length) {
        return -1; // Insufficient buffer size
    }

    double temperature, pressure, humidity;

    if (ESP_OK != bme280_read_temperature(&temperature) ||
        ESP_OK != bme280_read_pressure(&pressure) ||
        ESP_OK != bme280_read_humidity(&humidity)) {
        return -1;  // Error occurred while reading
    }

    // Store the data in the provided buffer
    memcpy(data, &temperature, sizeof(double));
    memcpy(data + sizeof(double), &pressure, sizeof(double));
    memcpy(data + 2 * sizeof(double), &humidity, sizeof(double));

    return 0;  // Success
}

int bme280_sensor_write(void* context, const uint8_t* data, size_t length) {
    // No specific write operations are necessary for BME280 in this context
    return 0; // Success
}

// Define the sensor interface for BME280 AFTER all functions are defined
sensor_interface_t bme280_interface = {
    .init = bme280_sensor_init,
    .read = bme280_sensor_read,
    .write = bme280_sensor_write,
};
