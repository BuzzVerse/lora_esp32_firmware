#ifndef I2C_DEFS_H
#define I2C_DEFS_H

#include "esp_err.h"
#include "driver/i2c.h"

typedef struct
{
    uint8_t i2c_address;                                                                       // I2C address of the sensor
    esp_err_t (*init)(void *sensor_config);                                                    // Initialization function
    esp_err_t (*read)(void *sensor_config, uint8_t reg_addr, uint8_t *data, size_t data_len);  // Read function
    esp_err_t (*write)(void *sensor_config, uint8_t reg_addr, uint8_t *data, size_t data_len); // Write function
    void *user_data;                                                                           // Pointer to additional sensor-specific data if needed
} sensor_config_t;

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL_IO // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA_IO // GPIO number for I2C master data
#define I2C_MASTER_NUM I2C_NUM_0                   // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 100000                  // I2C master clock frequency (100kHz)
#define I2C_MASTER_TX_BUF_DISABLE 0                // I2C master do not need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0                // I2C master do not need buffer
#define ACK_CHECK_EN 0x1                           // I2C master will check ack from slave
#define I2C_MASTER_TIMEOUT_MS 25                   // I2C timeout in milliseconds

#endif // I2C_DEFS_H