#include "bme280.h"
#include "bme280_lib.h"
#include "i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG_BME280 "BME280"

static struct bme280_t bme280;
#if 0
// Function to initialize I2C
esp_err_t i2c_master_init(void)
{
    esp_err_t esp_rc;

    // Configuration structure for I2C
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,                // I2C mode: Master
        .sda_io_num = CONFIG_I2C_MASTER_SDA_IO, // GPIO pin for SDA (data line)
        .scl_io_num = CONFIG_I2C_MASTER_SCL_IO, // GPIO pin for SCL (clock line)
        .sda_pullup_en = GPIO_PULLUP_ENABLE,    // Enable SDA pull-up resistor
        .scl_pullup_en = GPIO_PULLUP_ENABLE,    // Enable SCL pull-up resistor
        .master.clk_speed = 400000              // Clock speed (400 kHz)
    };

    gpio_set_direction(CONFIG_I2C_MASTER_ENABLED, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_I2C_MASTER_ENABLED, 1);

    // Configure I2C parameters
    esp_rc = i2c_param_config(I2C_NUM_0, &i2c_config);

    // Install the I2C driver
    esp_rc += i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    if (esp_rc == ESP_OK)
    {
        ESP_LOGI(TAG_BME280, "I2C init success");
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "I2C init failed. code: %d", esp_rc);
        return ESP_FAIL;
    }
}

esp_err_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    esp_err_t esp_rc;

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
    esp_rc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

    // Delete the I2C command link to free up resources
    i2c_cmd_link_delete(cmd);

    // Check if the write was successful
    if (esp_rc == ESP_OK)
    {
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "I2C write failed. code: %d", esp_rc);
        return ESP_FAIL;
    }
}

esp_err_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    esp_err_t esp_rc;

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
    esp_rc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

    // Delete the I2C command link to free up resources
    i2c_cmd_link_delete(cmd);

    // Check if the read was successful
    if (esp_rc == ESP_OK)
    {
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG_BME280, "I2C read failed. code: %d", esp_rc);
        return ESP_FAIL;
    }
}
#endif
// Function to provide millisecond delay
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
