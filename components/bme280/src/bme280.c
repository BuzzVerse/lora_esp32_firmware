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
    // Configuration structure for I2C
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,                // I2C mode: Master
        .sda_io_num = CONFIG_I2C_MASTER_SDA_IO, // GPIO pin for SDA (data line)
        .scl_io_num = CONFIG_I2C_MASTER_SCL_IO, // GPIO pin for SCL (clock line)
        .sda_pullup_en = GPIO_PULLUP_ENABLE,    // Enable SDA pull-up resistor
        .scl_pullup_en = GPIO_PULLUP_ENABLE,    // Enable SCL pull-up resistor
        .master.clk_speed = 1000000             // Clock speed (1 MHz)
    };

    // Configure I2C parameters
    i2c_param_config(I2C_NUM_0, &i2c_config);

    // Install the I2C driver
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int8_t iError = SUCCESS;
    esp_err_t espRc;

    // Create I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Write device address and register address for writing
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Write data to the device
    i2c_master_write(cmd, reg_data, cnt, true);

    // Stop the I2C communication
    i2c_master_stop(cmd);

    // Execute the I2C command sequence
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = 0; // Success
    }
    else
    {
        iError = -1; // Error
    }

    // Delete the I2C command link to free up resources
    i2c_cmd_link_delete(cmd);

    return iError; // Return the result
}

int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int8_t iError = SUCCESS;
    esp_err_t espRc;

    // Create I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Write device address and register address for reading
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Start a new I2C communication for reading
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    // Read data from the device
    if (cnt > 1)
    {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);

    // Stop the I2C communication
    i2c_master_stop(cmd);

    // Execute the I2C command sequence
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = 0; // Success
    }
    else
    {
        iError = -1; // Error
    }

    // Delete the I2C command link to free up resources
    i2c_cmd_link_delete(cmd);

    return iError; // Return the result
}

// Function to provide millisecond delay
void delay_ms(uint32_t ticks)
{
    vTaskDelay(ticks / portTICK_PERIOD_MS);
}

esp_err_t bme280_init_driver(uint8_t dev_addr)
{

    // Initialize the BME280 driver structure with I2C functions and the given address
    bme280.dev_addr = dev_addr;
    bme280.delay_msec = delay_ms;
    bme280.bus_write = i2c_write;
    bme280.bus_read = i2c_read;

    s32 com_rslt;

    // Initialize the I2C bus
    i2c_master_init();

    // Initialize the BME280 driver using BOSCH library
    com_rslt = bme280_init(&bme280);

    // Check if the initialization was successful
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

    // Map settings to actual values for the driver
    uint8_t oversampling_settings[6] = {
        BME280_OVERSAMP_SKIPPED,
        BME280_OVERSAMP_1X,
        BME280_OVERSAMP_2X,
        BME280_OVERSAMP_4X,
        BME280_OVERSAMP_8X,
        BME280_OVERSAMP_16X};

    oversamp_pressure = oversampling_settings[oversamp_pressure];
    oversamp_temperature = oversampling_settings[oversamp_temperature];
    oversamp_humidity = oversampling_settings[oversamp_humidity];

    // Set the oversampling
    com_rslt = bme280_set_oversamp_pressure(oversamp_pressure);
    com_rslt += bme280_set_oversamp_temperature(oversamp_temperature);
    com_rslt += bme280_set_oversamp_humidity(oversamp_humidity);

    // Check if the oversampling was set correctly
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

esp_err_t bme280_set_settings(uint8_t standby_time, uint8_t filter_coeff, uint8_t power_mode)
{
    // Result of communication results
    s32 com_rslt;

    // Map settings to actual values for the driver
    uint8_t standby_settings[8] = {
        BME280_STANDBY_TIME_1_MS,
        BME280_STANDBY_TIME_63_MS,
        BME280_STANDBY_TIME_125_MS,
        BME280_STANDBY_TIME_250_MS,
        BME280_STANDBY_TIME_500_MS,
        BME280_STANDBY_TIME_1000_MS,
        BME280_STANDBY_TIME_10_MS,
        BME280_STANDBY_TIME_20_MS};

    uint8_t filter_settings[5] = {
        BME280_FILTER_COEFF_OFF,
        BME280_FILTER_COEFF_2,
        BME280_FILTER_COEFF_4,
        BME280_FILTER_COEFF_8,
        BME280_FILTER_COEFF_16};

    uint8_t power_settings[3] = {
        BME280_SLEEP_MODE,
        BME280_FORCED_MODE,
        BME280_NORMAL_MODE};

    standby_time = standby_settings[standby_time];
    filter_coeff = filter_settings[filter_coeff];
    power_mode = power_settings[power_mode];

    // Set the settings
    com_rslt = bme280_set_standby_durn(standby_time);
    com_rslt += bme280_set_filter(filter_coeff);
    com_rslt += bme280_set_power_mode(power_mode);

    // Check if the settings were set correctly
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

double bme280_read_pressure(void)
{
    s32 com_rslt;

    // Uncompenstated pressure value
    s32 v_uncomp_pressure_s32;

    // Read the uncompensated pressure value
    com_rslt = bme280_read_uncomp_pressure(&v_uncomp_pressure_s32);

    // Check if the read was successful
    if (com_rslt == SUCCESS)
    {
        // Compensate the pressure value and return it
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

    // Uncompenstated temperature value
    s32 v_uncomp_temperature_s32;

    // Read the uncompensated temperature value
    com_rslt = bme280_read_uncomp_temperature(&v_uncomp_temperature_s32);

    // Check if the read was successful
    if (com_rslt == SUCCESS)
    {
        // Compensate the temperature value and return it
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

    // Uncompenstated humidity value
    s32 v_uncomp_humidity_s32;

    // Read the uncompensated humidity value
    com_rslt = bme280_read_uncomp_humidity(&v_uncomp_humidity_s32);

    // Check if the read was successful
    if (com_rslt == SUCCESS)
    {
        // Compensate the humidity value and return it
        return bme280_compensate_humidity_double(v_uncomp_humidity_s32);
    }
    else
    {
        ESP_LOGE(TAG_BME280, "BME280 uncomp humidity read failed. code: %d", com_rslt);
        return 0;
    }
}