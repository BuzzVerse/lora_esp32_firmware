#include "api/driver_api.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define SPI_WRITE_OPERATION 0x80
#define SPI_DEFAULT_BYTE 0xff
#define SPI_DELAY_100MS 100
#define MAX_BUFFER_LEN 128

static spi_device_handle_t __spi;
static const char *LORA_API_TAG = "LORA_API";

api_status_t spi_init(void)
{
    ESP_LOGI(LORA_API_TAG, "Initializing spi...\n");
    esp_err_t ret;

    ret = gpio_reset_pin(CONFIG_RST_GPIO);
    ret += gpio_set_direction(CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);

    if (ESP_OK != ret)
        return API_FAILED_SPI_SET_PIN;

    lora_reset();

    ret += gpio_reset_pin(CONFIG_CS_GPIO);
    ret += gpio_set_direction(CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);

    if (ESP_OK != ret)
        return API_FAILED_SPI_SET_PIN;

    ret += gpio_set_level(CONFIG_CS_GPIO, 1);

    if (ESP_OK != ret)
        return API_FAILED_SPI_CHIP_SELECT;

    spi_bus_config_t bus = {
        .miso_io_num = CONFIG_MISO_GPIO,
        .mosi_io_num = CONFIG_MOSI_GPIO,
        .sclk_io_num = CONFIG_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0};

    ret += spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);

    if (ESP_OK != ret)
        return API_FAILED_SPI_INIT;

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 9000000,
        .mode = 0,
        .spics_io_num = CONFIG_CS_GPIO,
        .queue_size = 7,
        .flags = 0,
        .pre_cb = NULL};

    ret += spi_bus_add_device(SPI2_HOST, &dev, &__spi);

    if (ESP_OK != ret)
        return API_FAILED_SPI_ADD_DEVICE;

    ESP_LOGI(LORA_API_TAG, "SPI initialized successfully");
    return API_OK;
}

api_status_t spi_write(uint8_t reg, uint8_t val)
{
    uint8_t out[2] = {SPI_WRITE_OPERATION | reg, val};
    uint8_t in[2];

    spi_transaction_t transaction = {
        .flags = 0,
        .length = 8 * sizeof(out),
        .tx_buffer = out,
        .rx_buffer = in};

    if (ESP_OK != spi_device_transmit(__spi, &transaction))
    {
        ESP_LOGE(LORA_API_TAG, "SPI write failed: reg=0x%02X, val=0x%02X", reg, val);
        return API_FAILED_SPI_WRITE;
    }

    return API_OK;
}

api_status_t spi_write_buf(uint8_t reg, uint8_t *val, uint8_t len)
{
    if (NULL == val)
    {
        ESP_LOGE(LORA_API_TAG, "Cannot assign value to NULL pointer");
        return API_NULL_POINTER_ERROR;
    }

    if (len > MAX_BUFFER_LEN)
    {
        ESP_LOGE(LORA_API_TAG, "Buffer length exceeds maximum allowed size");
        return API_BUFFER_TOO_LARGE;
    }

    uint8_t out[MAX_BUFFER_LEN + 1];

    out[0] = SPI_WRITE_OPERATION | reg;

    for (uint8_t i = 0; i < len; i++)
    {
        out[i + 1] = val[i];
    }

    spi_transaction_t transaction = {
        .flags = 0,
        .length = 8 * (len + 1),
        .tx_buffer = out,
        .rx_buffer = NULL};

    if (ESP_OK != spi_device_transmit(__spi, &transaction))
    {
        ESP_LOGE(LORA_API_TAG, "SPI buffer write failed: reg=0x%02X, len=%d", reg, len);
        return API_FAILED_SPI_WRITE_BUF;
    }

    ESP_LOGD(LORA_API_TAG, "SPI buffer write successful: reg=0x%02X, len=%d", reg, len);
    return API_OK;
}

api_status_t spi_read(uint8_t reg, uint8_t *val)
{
    if (NULL == val)
    {
        ESP_LOGE(LORA_API_TAG, "Cannot assign value to NULL pointer");
        return API_NULL_POINTER_ERROR;
    }

    uint8_t out[2] = {reg, SPI_DEFAULT_BYTE};
    uint8_t in[2];

    spi_transaction_t transaction = {
        .flags = 0,
        .length = 8 * sizeof(out),
        .tx_buffer = out,
        .rx_buffer = in};

    if (ESP_OK != spi_device_transmit(__spi, &transaction))
    {
        ESP_LOGE(LORA_API_TAG, "SPI read failed: reg=0x%02X", reg);
        return API_FAILED_SPI_READ;
    }

    *val = in[1];
    return API_OK;
}

api_status_t spi_read_buf(uint8_t reg, uint8_t *val, uint8_t len)
{
    if (NULL == val)
    {
        ESP_LOGE(LORA_API_TAG, "Cannot assign value to NULL pointer");
        return API_NULL_POINTER_ERROR;
    }

    if (len > MAX_BUFFER_LEN)
    {
        ESP_LOGE(LORA_API_TAG, "Buffer length exceeds maximum allowed size");
        return API_BUFFER_TOO_LARGE;
    }

    uint8_t out[MAX_BUFFER_LEN + 1];
    uint8_t in[MAX_BUFFER_LEN + 1];

    out[0] = reg;

    for (uint8_t i = 0; i < len; i++)
    {
        out[i + 1] = SPI_DEFAULT_BYTE;
    }

    spi_transaction_t transaction = {
        .flags = 0,
        .length = 8 * (len + 1),
        .tx_buffer = out,
        .rx_buffer = in};

    if (ESP_OK != spi_device_transmit(__spi, &transaction))
    {
        ESP_LOGE(LORA_API_TAG, "SPI buffer read failed: reg=0x%02X, len=%d", reg, len);
        return API_FAILED_SPI_READ_BUF;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        val[i] = in[i + 1];
    }

    ESP_LOGD(LORA_API_TAG, "SPI buffer read successful: reg=0x%02X, len=%d", reg, len);
    return API_OK;
}

api_status_t lora_reset(void)
{
    api_status_t ret = API_OK;
    ESP_LOGI(LORA_API_TAG, "Resetting LoRa module...");

    ret += gpio_set_level(CONFIG_RST_GPIO, 0);
    lora_delay(SPI_DELAY_100MS);

    ret += gpio_set_level(CONFIG_RST_GPIO, 1);
    lora_delay(SPI_DELAY_100MS);

    if (ESP_OK != ret)
    {
        ESP_LOGE(LORA_API_TAG, "Failed to reset LoRa module");
        return API_FAILED_SPI_SET_LEVEL;
    }
    ESP_LOGI(LORA_API_TAG, "LoRa module reset!");

    return API_OK;
}

void lora_delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}