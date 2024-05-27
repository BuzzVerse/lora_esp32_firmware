#include "api/driver_api.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

static spi_device_handle_t __spi;
static const char *LORA_API_TAG = "LORA_API";

api_status_t spi_init(void)
{
    ESP_LOGI(LORA_API_TAG, "Initializing spi...\n");
    esp_err_t ret;

    ret = gpio_reset_pin(CONFIG_RST_GPIO);
    ret += gpio_set_direction(CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);

    lora_reset();

    ret += gpio_reset_pin(CONFIG_CS_GPIO);
    ret += gpio_set_direction(CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);
    ret += gpio_set_level(CONFIG_CS_GPIO, 1);

    spi_bus_config_t bus = {
        .miso_io_num = CONFIG_MISO_GPIO,
        .mosi_io_num = CONFIG_MOSI_GPIO,
        .sclk_io_num = CONFIG_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0};

    ret += spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 9000000,
        .mode = 0,
        .spics_io_num = CONFIG_CS_GPIO,
        .queue_size = 7,
        .flags = 0,
        .pre_cb = NULL};

    ret += spi_bus_add_device(SPI2_HOST, &dev, &__spi);

    if (ESP_OK == ret)
    {
        ESP_LOGI(LORA_API_TAG, "SPI initialized successfully");
        return API_OK;
    }
    else
    {
        ESP_LOGE(LORA_API_TAG, "SPI initialization failed");
        return API_SPI_ERROR;
    }
}

api_status_t spi_write(uint8_t reg, uint8_t val)
{
    uint8_t out[2] = {0x80 | reg, val};
    uint8_t in[2];

    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * sizeof(out),
        .tx_buffer = out,
        .rx_buffer = in};

    if (ESP_OK == spi_device_transmit(__spi, &t))
    {
        return API_OK;
    }
    else
    {
        ESP_LOGE(LORA_API_TAG, "SPI write failed: reg=0x%02X, val=0x%02X", reg, val);
        return API_SPI_ERROR;
    }
}

api_status_t spi_write_buf(uint8_t reg, uint8_t *val, uint16_t len)
{
    uint8_t *out;
    esp_err_t ret;

    out = (uint8_t *)malloc(len + 1);
    out[0] = 0x80 | reg;
    for (uint8_t i = 0; i < len; i++)
    {
        out[i + 1] = val[i];
    }

    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * (len + 1),
        .tx_buffer = out,
        .rx_buffer = NULL};

    ret = spi_device_transmit(__spi, &t);
    free(out);

    if (ESP_OK == ret)
    {
        ESP_LOGD(LORA_API_TAG, "SPI buffer write successful: reg=0x%02X, len=%d", reg, len);
        return API_OK;
    }
    else
    {
        ESP_LOGE(LORA_API_TAG, "SPI buffer write failed: reg=0x%02X, len=%d", reg, len);
        return API_SPI_ERROR;
    }
}

api_status_t spi_read(uint8_t reg, uint8_t *val)
{
    uint8_t out[2] = {reg, 0xff};
    uint8_t in[2];
    esp_err_t ret;

    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * sizeof(out),
        .tx_buffer = out,
        .rx_buffer = in};

    ret = spi_device_transmit(__spi, &t);
    *val = in[1];

    if (ESP_OK == ret)
    {
        return API_OK;
    }
    else
    {
        ESP_LOGE(LORA_API_TAG, "SPI read failed: reg=0x%02X", reg);
        return API_SPI_ERROR;
    }
}

api_status_t spi_read_buf(uint8_t reg, uint8_t *val, uint16_t len)
{
    uint8_t *out;
    uint8_t *in;
    esp_err_t ret;

    out = (uint8_t *)malloc(len + 1);
    in = (uint8_t *)malloc(len + 1);
    out[0] = reg;

    for (uint8_t i = 0; i < len; i++)
    {
        out[i + 1] = 0xff;
    }

    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * (len + 1),
        .tx_buffer = out,
        .rx_buffer = in};

    ret = spi_device_transmit(__spi, &t);

    for (uint8_t i = 0; i < len; i++)
    {
        val[i] = in[i + 1];
    }

    free(out);
    free(in);

    if (ESP_OK == ret)
    {
        ESP_LOGD(LORA_API_TAG, "SPI buffer read successful: reg=0x%02X, len=%d", reg, len);
        return API_OK;
    }
    else
    {
        ESP_LOGE(LORA_API_TAG, "SPI buffer read failed: reg=0x%02X, len=%d", reg, len);
        return API_SPI_ERROR;
    }
}

void lora_reset(void)
{
    ESP_LOGI(LORA_API_TAG, "Resetting LoRa module...");
    gpio_set_level(CONFIG_RST_GPIO, 0);
    lora_delay(100);
    gpio_set_level(CONFIG_RST_GPIO, 1);
    lora_delay(100);
    ESP_LOGI(LORA_API_TAG, "LoRa module reset!");
}

void lora_delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}