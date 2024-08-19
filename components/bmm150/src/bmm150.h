#ifndef BMM150_H
#define BMM150_H

#include "bmm150_defs.h"
#include "bmm150_lib.h"
#include "esp_err.h"

esp_err_t bmm150_init_driver(uint8_t dev_addr);
esp_err_t bmm150_read_mag_data_driver(struct bmm150_mag_data *mag_data);

esp_err_t setup_bmm150_normal_mode(struct bmm150_dev *dev);
esp_err_t set_config(struct bmm150_settings *dev);

#endif // BMM150_H