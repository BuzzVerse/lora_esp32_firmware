#ifndef BQ27441_H
#define BQ27441_H

#include "esp_err.h"

#define BQ27441_ADDR 0x55 // 7-bit address of the BQ27441 fuel gauge

esp_err_t bq27441_init(void);
esp_err_t bq27441_set_design_capacity(uint16_t capacity);
esp_err_t bq27441_set_design_energy(uint16_t energy);
esp_err_t bq27441_set_terminate_voltage(uint16_t voltage);
esp_err_t bq27441_set_taper_rate(uint16_t rate);
esp_err_t bq27441_read_design_capacity(uint16_t *capacity);
esp_err_t bq27441_read_soc(uint8_t *soc);
esp_err_t bq27441_read_voltage(uint16_t *voltage);

#endif // BQ27441_H
