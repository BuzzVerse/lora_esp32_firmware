#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

// Structure holding function pointers for sensor operations
typedef struct
{
    esp_err_t (*init)(void *context);
    esp_err_t (*read)(void *context, uint8_t *data, const size_t length);
    esp_err_t (*write)(void *context, const uint8_t *data, const size_t length);
} sensor_interface_t;

// Structure holding sensor context information
typedef struct
{
    const sensor_interface_t *interface;
    void *driver_data; // Pointer to sensor-specific data, e.g., I2C address, configuration, etc.
    void *sensor_data; // Pointer to sensor-specific data, e.g., calibration data, etc.
} sensor_context_t;

// Function to initialize the sensor
static inline esp_err_t sensor_init(sensor_context_t *ctx, const sensor_interface_t *interface, void *driver_data)
{
    ctx->interface = interface;
    ctx->driver_data = driver_data;
    return ctx->interface->init(ctx);
}

// Function to read data from the sensor
static inline esp_err_t sensor_read(sensor_context_t *ctx, uint8_t *data, size_t length)
{
    return ctx->interface->read(ctx, data, length);
}

// Function to write data to the sensor
static inline esp_err_t sensor_write(sensor_context_t *ctx, const uint8_t *data, size_t length)
{
    return ctx->interface->write(ctx, data, length);
}

#endif // _SENSOR_H_
