#include "sensor.h"
#include "bq27441.h"
#include "bme280.h"
#include "esp_log.h"
#include "i2c.h"
#include <string.h>

#define TAG_MAIN "Main"

static bq27441_config_t bq_config;
static sensor_context_t bq_sensor_context;

static sensor_context_t bme280_sensor_context;

static void initialize_i2c(void) {
    i2c_init();
}

static void configure_bq27441(void) {
    bq_config.i2c_address = CONFIG_BQ27441_I2C_ADDRESS;
    bq_config.design_capacity = CONFIG_BQ27441_DESIGN_CAPACITY;
    bq_config.terminate_voltage = CONFIG_BQ27441_TERMINATE_VOLTAGE;

    bq_sensor_context.interface = &bq27441_interface;
    bq_sensor_context.driver_data = &bq_config;
}

static void configure_bme280(void) {
    bme280_sensor_context.interface = &bme280_interface;
    bme280_sensor_context.driver_data = (void *)CONFIG_BME280_I2C_ADDRESS; // Assuming driver_data holds I2C address
}

static void initialize_sensors(void) {
    if (sensor_init(&bq_sensor_context, &bq27441_interface, &bq_config) != 0) {
        ESP_LOGE(TAG_MAIN, "Failed to initialize BQ27441");
        esp_restart();
    }

    if (sensor_init(&bme280_sensor_context, &bme280_interface, bme280_sensor_context.driver_data) != 0) {
        ESP_LOGE(TAG_MAIN, "Failed to initialize BME280");
        esp_restart();
    }

    // Set BME280 sensor settings
    if (bme280_set_oversamp(OVERSAMP_16X, OVERSAMP_16X, OVERSAMP_16X) != ESP_OK ||
        bme280_set_settings(STANDBY_10MS, FILTER_16, MODE_NORMAL) != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to set BME280 settings");
        esp_restart();
    }
}

static void read_bq27441_data(void) {
    uint8_t bq27441_data[3 * sizeof(uint16_t)];
    if (sensor_read(&bq_sensor_context, bq27441_data, sizeof(bq27441_data)) == 0) {
        uint16_t capacity;
        uint8_t soc;
        uint16_t voltage;

        memcpy(&capacity, bq27441_data, sizeof(uint16_t));
        memcpy(&soc, bq27441_data + sizeof(uint16_t), sizeof(uint8_t));
        memcpy(&voltage, bq27441_data + sizeof(uint16_t) + sizeof(uint8_t), sizeof(uint16_t));

        ESP_LOGI(TAG_MAIN, "BQ27441 - Design Capacity: %d mAh", capacity);
        ESP_LOGI(TAG_MAIN, "BQ27441 - State of Charge: %d%%", soc);
        ESP_LOGI(TAG_MAIN, "BQ27441 - Voltage: %d mV", voltage);
    } else {
        ESP_LOGE(TAG_MAIN, "Failed to read BQ27441 data");
    }
}

static void read_bme280_data(void) {
    uint8_t bme280_data[3 * sizeof(double)]; // Buffer to hold temperature, pressure, and humidity

    if (sensor_read(&bme280_sensor_context, bme280_data, sizeof(bme280_data)) == 0) {
        double temperature, pressure, humidity;
        memcpy(&temperature, bme280_data, sizeof(double));
        memcpy(&pressure, bme280_data + sizeof(double), sizeof(double));
        memcpy(&humidity, bme280_data + 2 * sizeof(double), sizeof(double));

        ESP_LOGI(TAG_MAIN, "BME280 - Temperature: %.2f C", temperature);
        ESP_LOGI(TAG_MAIN, "BME280 - Pressure: %.2f hPa", pressure / 100.0);
        ESP_LOGI(TAG_MAIN, "BME280 - Humidity: %.2f %%", humidity);
    } else {
        ESP_LOGE(TAG_MAIN, "Failed to read BME280 data");
    }
}

void app_main(void) {
    initialize_i2c();
    configure_bq27441();
    configure_bme280();
    initialize_sensors();

    // Initial data read
    read_bq27441_data();
    read_bme280_data();

    // Periodic data read
    while (1) {
        read_bme280_data();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
