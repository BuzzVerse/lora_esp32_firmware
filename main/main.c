#include "sensor.h"
#include "bq27441.h"
#include "bme280.h"
#include "i2c.h"

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <string.h>

#define TAG_MAIN "Main"

// static bq27441_config_t bq_config;
// static sensor_context_t bq_sensor_context;
// static i2c_master_dev_handle_t bq_dev_handle;

static sensor_context_t bme280_sensor_context;
static i2c_master_dev_handle_t bme280_dev_handle;

static void initialize_i2c(void)
{
    i2c_init();

    i2c_device_init(&bme280_interface.config.i2c.config, bme280_sensor_context.interface->config.i2c.dev_handle);
}

// static void configure_bq27441(void)
// {
//     bq_config.i2c_address = CONFIG_BQ27441_I2C_ADDRESS;
//     bq_config.design_capacity = CONFIG_BQ27441_DESIGN_CAPACITY;
//     bq_config.terminate_voltage = CONFIG_BQ27441_TERMINATE_VOLTAGE;

//     bq_sensor_context.interface = &bq27441_interface;
//     bq_sensor_context.driver_data = &bq_config;
// }

static void configure_bme280(void)
{
}

static void initialize_sensors(void)
{
    // if (sensor_init(&bq_sensor_context, &bq27441_interface, &bq_config) != 0)
    // {
    //     ESP_LOGE(TAG_MAIN, "Failed to initialize BQ27441");
    //     esp_restart();
    // }

    ESP_LOGI(TAG_MAIN, "bme280_interface.config.i2c.dev_handle: %p", bme280_interface.config.i2c.dev_handle);

    if (sensor_init(&bme280_sensor_context, &bme280_interface, bme280_sensor_context.driver_data) != 0)
    {
        ESP_LOGE(TAG_MAIN, "Failed to initialize BME280");
        esp_restart();
    }

    // Set BME280 sensor settings
    if (bme280_set_oversamp(OVERSAMP_16X, OVERSAMP_16X, OVERSAMP_16X) != ESP_OK ||
        bme280_set_settings(STANDBY_10MS, FILTER_16, MODE_NORMAL) != ESP_OK)
    {
        ESP_LOGE(TAG_MAIN, "Failed to set BME280 settings");
        esp_restart();
    }
}

// static void read_bq27441_data(void)
// {
//     uint8_t bq27441_data[3 * sizeof(uint16_t)];
//     if (sensor_read(&bq_sensor_context, bq27441_data, sizeof(bq27441_data)) == 0)
//     {
//         uint16_t capacity;
//         uint8_t soc;
//         uint16_t voltage;

//         memcpy(&capacity, bq27441_data, sizeof(uint16_t));
//         memcpy(&soc, bq27441_data + sizeof(uint16_t), sizeof(uint8_t));
//         memcpy(&voltage, bq27441_data + sizeof(uint16_t) + sizeof(uint8_t), sizeof(uint16_t));

//         ESP_LOGI(TAG_MAIN, "BQ27441 - Design Capacity: %d mAh", capacity);
//         ESP_LOGI(TAG_MAIN, "BQ27441 - State of Charge: %d%%", soc);
//         ESP_LOGI(TAG_MAIN, "BQ27441 - Voltage: %d mV", voltage);
//     }
//     else
//     {
//         ESP_LOGE(TAG_MAIN, "Failed to read BQ27441 data");
//     }
// }

static void read_bme280_data(void)
{
    uint8_t bme280_data[3 * sizeof(double)]; // Buffer to hold temperature, pressure, and humidity

    if (sensor_read(&bme280_sensor_context, bme280_data, sizeof(bme280_data)) == 0)
    {
        double temperature, pressure, humidity;
        memcpy(&temperature, bme280_data, sizeof(double));
        memcpy(&pressure, bme280_data + sizeof(double), sizeof(double));
        memcpy(&humidity, bme280_data + 2 * sizeof(double), sizeof(double));

        ESP_LOGI(TAG_MAIN, "BME280 - Temperature: %.2f C", temperature);
        ESP_LOGI(TAG_MAIN, "BME280 - Pressure: %.2f hPa", pressure / 100.0);
        ESP_LOGI(TAG_MAIN, "BME280 - Humidity: %.2f %%", humidity);
    }
    else
    {
        ESP_LOGE(TAG_MAIN, "Failed to read BME280 data");
    }
}

void app_main(void)
{
    // configure_bq27441();
    // configure_bme280();

    // initialize_i2c();

    bme280_sensor_context.interface = &bme280_interface;
    bme280_sensor_context.interface->config.i2c.dev_handle = &bme280_dev_handle; // Set the device handle as driver data

    ESP_LOGI(TAG_MAIN, "Initializing i2c");
    i2c_init();

    ESP_LOGI(TAG_MAIN, "Initializing BME280");

    esp_err_t err = i2c_device_init(&bme280_interface.config.i2c.config, bme280_sensor_context.interface->config.i2c.dev_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MAIN, "Failed initialize device handle: %s", esp_err_to_name(err));
        esp_restart();
    }

    ESP_LOGI(TAG_MAIN, "Initializing sensors");

    initialize_sensors();

    // Initial data read
    // read_bq27441_data();
    read_bme280_data();

    // Periodic data read
    while (1)
    {
        read_bme280_data();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
