#include "bmm150.h"
#include "bmm150_lib.h"
#include "i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG_BMM150 "BMM150"



int8_t bmm150_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t length, void *intf_ptr) {
    uint8_t device_addr = *((uint8_t*)intf_ptr); // Assuming intf_ptr holds the device address
    return (int8_t)i2c_read(device_addr, reg_addr, data, length);
}

int8_t bmm150_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t length, void *intf_ptr) {
    uint8_t device_addr = *((uint8_t*)intf_ptr);
    return (int8_t)i2c_write(device_addr, reg_addr, (uint8_t *)data, length);
}

void bmm150_delay_us(uint32_t period, void *intf_ptr) {
    delay_ms(period / 1000); // Convert microseconds to milliseconds
}

esp_err_t setup_bmm150_normal_mode(struct bmm150_dev *dev) {
    int8_t rslt;
    struct bmm150_settings settings;

    // Set the power mode to normal
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, dev);
    if (rslt != BMM150_OK) {
        return rslt;
    }

    // Set the desired ODR and repetition values for normal mode
    settings.data_rate = BMM150_DATA_RATE_10HZ;  // Set desired data rate
    settings.xy_rep = BMM150_REPXY_REGULAR;      // XY-axis repetition for normal mode
    settings.z_rep = BMM150_REPZ_REGULAR;        // Z-axis repetition for normal mode
    rslt = set_odr_xyz_rep(&settings, dev);
    if (rslt != BMM150_OK) {
        return rslt;
    }

    // Enable measurement for XYZ axes (ensure correct usage of control bits)
    settings.xyz_axes_control = BMM150_CONTROL_MEASURE_POS; // Correct constant
    rslt = set_control_measurement_xyz(&settings, dev);

    return rslt;
}

esp_err_t bmm150_init_driver(uint8_t dev_addr){
    struct bmm150_dev dev;
    dev.intf = BMM150_I2C_INTF;
    dev.intf_ptr = &dev_addr;
    dev.read = bmm150_i2c_read;
    dev.write = bmm150_i2c_write;
    dev.delay_us = bmm150_delay_us;
    esp_err_t ret = bmm150_init(&dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_BMM150, "BMM150 init failed.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG_BMM150, "BMM150 init success");
    return ESP_OK;
}

esp_err_t bmm150_read_mag_data_driver(struct bmm150_mag_data *mag_data, struct bmm150_dev *dev) {
    int8_t rslt = bmm150_read_mag_data(mag_data, dev);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG_BMM150, "Error reading magnetometer data: %d", rslt);
        return ESP_FAIL;
    }
    return ESP_OK;
}

// esp_err_t bmm150_read_mag_data_esp(struct bmm150_mag_data *mag_data, struct bmm150_dev *dev) {
//     esp_err_t ret = ESP_OK;
//     int16_t msb_data;
//     uint8_t reg_data[BMM150_LEN_XYZR_DATA] = { 0 };
//     struct bmm150_raw_mag_data raw_mag_data;

//     // Read the mag data registers
//     int8_t rslt = bmm150_get_regs(BMM150_REG_DATA_X_LSB, reg_data, BMM150_LEN_XYZR_DATA, dev);
//     if (rslt != BMM150_OK) {
//         ESP_LOGE(TAG_BMM150, "Failed to read magnetometer registers, error code: %d", rslt);
//         return ESP_FAIL;
//     }

//     // Mag X axis data
//     reg_data[0] = BMM150_GET_BITS(reg_data[0], BMM150_DATA_X);
//     msb_data = ((int16_t)((int8_t)reg_data[1])) * 32;
//     raw_mag_data.raw_datax = (int16_t)(msb_data | reg_data[0]);

//     // Mag Y axis data
//     reg_data[2] = BMM150_GET_BITS(reg_data[2], BMM150_DATA_Y);
//     msb_data = ((int16_t)((int8_t)reg_data[3])) * 32;
//     raw_mag_data.raw_datay = (int16_t)(msb_data | reg_data[2]);

//     // Mag Z axis data
//     reg_data[4] = BMM150_GET_BITS(reg_data[4], BMM150_DATA_Z);
//     msb_data = ((int16_t)((int8_t)reg_data[5])) * 128;
//     raw_mag_data.raw_dataz = (int16_t)(msb_data | reg_data[4]);

//     // Mag R-HALL data
//     reg_data[6] = BMM150_GET_BITS(reg_data[6], BMM150_DATA_RHALL);
//     raw_mag_data.raw_data_r = (uint16_t)(((uint16_t)reg_data[7] << 6) | reg_data[6]);

//     // Compensated Mag X data in int16_t format
//     mag_data->x = compensate_x(raw_mag_data.raw_datax, raw_mag_data.raw_data_r, dev);

//     // Compensated Mag Y data in int16_t format
//     mag_data->y = compensate_y(raw_mag_data.raw_datay, raw_mag_data.raw_data_r, dev);

//     // Compensated Mag Z data in int16_t format
//     mag_data->z = compensate_z(raw_mag_data.raw_dataz, raw_mag_data.raw_data_r, dev);

//     return ESP_OK;
// }

// esp_err_t write_op_mode(uint8_t op_mode, struct bmm150_dev *dev) {
//     int8_t rslt = bmm150_set_op_mode(op_mode, dev);
//     if (rslt != BMM150_OK) {
//         ESP_LOGE(TAG_BMM150, "Error setting op mode: %d", rslt);
//         return ESP_FAIL;
//     }
//     return ESP_OK;
// }

// int8_t interrupt_threshold_settings(uint16_t desired_settings,
//                                     const struct bmm150_settings *settings,
//                                     struct bmm150_dev *dev);

// esp_err_t set_interrupt_settings(uint16_t desired_settings, const struct bmm150_settings *settings, struct bmm150_dev *dev) {
//     int8_t rslt = interrupt_threshold_settings(desired_settings, settings, dev);
//     if (rslt != BMM150_OK) {
//         ESP_LOGE(TAG_BMM150, "Error setting interrupt threshold settings: %d", rslt);
//         return ESP_FAIL;
//     }
//     return ESP_OK;
// }