#ifndef SPI_API_H
#define SPI_API_H

#include <stdint.h>

typedef enum {
    SPI_OK,
    SPI_ERROR
} spi_status_t;

spi_status_t spi_init(void);

spi_status_t spi_write(uint8_t reg, uint8_t val);

spi_status_t spi_write_buf(uint8_t reg, uint8_t *val, uint16_t len);

spi_status_t spi_read(uint8_t reg, uint8_t *val);

spi_status_t spi_read_buf(uint8_t reg, uint8_t *val, uint16_t len);

#endif // SPI_API_H