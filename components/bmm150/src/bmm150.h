#ifndef BMM150_H
#define BMM150_H

#include "bmm150_defs.h"
#include "bmm150_lib.h"
#include "esp_err.h"

esp_err_t bmm150_init_driver(uint8_t dev_addr);

#endif // BMM150_H