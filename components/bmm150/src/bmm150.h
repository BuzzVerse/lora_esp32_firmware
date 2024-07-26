#ifndef BMM150_H
#define BMM150_H

#include "bmm150_defs.h"
#include "bmm150_lib.h"
#include "esp_err.h"

esp_err_t bmm150_init_driver(uint8_t dev_addr);
esp_err_t bmm150_read_mag_data_driver(struct bmm150_mag_data *mag_data, struct bmm150_dev *dev);
int8_t interrupt_threshold_settings(uint16_t desired_settings,
                                           const struct bmm150_settings *settings,
                                           struct bmm150_dev *dev);



//  *  op_mode    |  Power mode
//  * ------------|-----------------------
//  *   0x00      |   BMM150_POWERMODE_NORMAL
//  *   0x01      |   BMM150_POWERMODE_FORCED
//  *   0x03      |   BMM150_POWERMODE_SLEEP

esp_err_t write_op_mode(uint8_t op_mode, struct bmm150_dev *dev);

#endif // BMM150_H