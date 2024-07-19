#ifndef BQ27441_H
#define BQ27441_H

#include "esp_err.h"

#define BQ27441_ADDR 0x55 // 7-bit address of the BQ27441 fuel gauge

esp_err_t bq27441_read_design_capacity(uint16_t *capacity);
esp_err_t bq27441_read_soc(uint16_t *soc);
esp_err_t bq27441_read_voltage(uint16_t *voltage);

#endif // BQ27441_H
