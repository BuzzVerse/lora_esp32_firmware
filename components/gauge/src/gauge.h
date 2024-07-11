#ifndef GAUGE_H
#define GAUGE_H
#define GAUGE_ADDR 0x55
#define BQ27441_DEVICE_ID 0x0421

esp_err_t get_voltage(uint16_t *voltage);

#endif // GAUGE_H