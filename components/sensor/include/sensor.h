#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <stdint.h>
#include <stddef.h>

// Structure holding function pointers for sensor operations
typedef struct {
    int (*init)(void* context);
    int (*read)(void* context, uint8_t* data, size_t length);
    int (*write)(void* context, const uint8_t* data, size_t length);
} sensor_interface_t;

// Structure holding sensor context information
typedef struct {
    const sensor_interface_t* interface;
    void* driver_data; // Pointer to sensor-specific data, e.g., I2C address, configuration, etc.
} sensor_context_t;

// Function to initialize the sensor
static inline int sensor_init(sensor_context_t* ctx, const sensor_interface_t* interface, void* driver_data) {
    ctx->interface = interface;
    ctx->driver_data = driver_data;
    return ctx->interface->init(ctx);
}

// Function to read data from the sensor
static inline int sensor_read(sensor_context_t* ctx, uint8_t* data, size_t length) {
    return ctx->interface->read(ctx, data, length);
}

// Function to write data to the sensor
static inline int sensor_write(sensor_context_t* ctx, const uint8_t* data, size_t length) {
    return ctx->interface->write(ctx, data, length);
}

#endif // _SENSOR_H_
