#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "bme280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lora_driver.h"

#define CONFIG_433MHZ 1
static uint8_t tx_buf[24];
static uint8_t rx_buf[24];

typedef union
{
    double value;
    uint8_t serialized[8];

} serialized_data_t;

typedef struct
{
    serialized_data_t temperature;
    serialized_data_t humidity;
    serialized_data_t pressure;

} bme280_data_t;

static bme280_data_t data;

#if CONFIG_LORA_TRANSMITTER
void task_tx(void *pvParameters)
{
    double temp = 0, pres = 0, hum = 0;
    ESP_LOGI(pcTaskGetName(NULL), "Start");

    vTaskDelay(pdMS_TO_TICKS(5000));

    while (1)
    {
        bme280_read_temperature(&temp);
        printf("temp: %f", temp);
        vTaskDelay(100);
        bme280_read_pressure(&pres);
        printf("pres: %f", pres);
        vTaskDelay(100);
        bme280_read_humidity(&hum);
        printf("hum: %f", hum);
        vTaskDelay(100);

        data.temperature.value = temp;
        data.pressure.value = pres;
        data.humidity.value = hum;

        ESP_LOGI(pcTaskGetName(NULL), "Temperature: %.2f C", data.temperature.value);
        ESP_LOGI(pcTaskGetName(NULL), "Pressure: %.2f hPa", data.pressure.value / 100);
        ESP_LOGI(pcTaskGetName(NULL), "Humidity: %.2f %%", data.humidity.value);

        for (int i = 0; i < 8; i++)
        {
            tx_buf[i] = data.temperature.serialized[i];
        }

        for (int i = 8; i < 16; i++)
        {
            tx_buf[i] = data.pressure.serialized[i - 8];
        }

        for (int i = 16; i < 24; i++)
        {
            tx_buf[i] = data.humidity.serialized[i - 16];
        }

        lora_send_packet(tx_buf, 24);

        int lost = lora_packet_lost();

        if (lost != 0)
        {
            ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
#endif

#if CONFIG_LORA_RECEIVER
void task_rx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    bme280_data_t data;

    while (1)
    {
        lora_receive(); // put into receive mode
        bool hasReceived;
        lora_received(&hasReceived);

        if (hasReceived)
        {
            uint8_t rxLen = 0;
            lora_receive_packet(rx_buf, &rxLen, sizeof(rx_buf));

            for (int i = 0; i < 24; i++)
            {
                printf("%d: 0x%x \n", i, rx_buf[i]);
            }

            for (int i = 0; i < 8; i++)
            {
                data.temperature.serialized[i] = rx_buf[i];
            }

            for (int i = 8; i < 16; i++)
            {
                data.pressure.serialized[i - 8] = rx_buf[i];
            }

            for (int i = 16; i < 24; i++)
            {
                data.humidity.serialized[i - 16] = rx_buf[i];
            }

            ESP_LOGI(pcTaskGetName(NULL), "Temp: %f, Pres: %f %% Hum: %f", data.temperature.value, data.pressure.value, data.humidity.value);
        }
        vTaskDelay(1); // Avoid WatchDog alerts
    }                  // end while
}
#endif

// Main application
void app_main(void)
{
    if (ESP_OK != lora_init())
    {
        ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
        while (1)
        {
            vTaskDelay(1);
        }
    }

#if CONFIG_LORA_TRANSMITTER

    esp_err_t bme_rc = ESP_OK;
    bme_rc += bme280_init_driver(CONFIG_BME280_I2C_ADDRESS);

    bme_rc += bme280_set_oversamp(
        BME280_OVERSAMP_16X,
        BME280_OVERSAMP_16X,
        BME280_OVERSAMP_16X);

    bme_rc += bme280_set_settings(
        STANDBY_10MS,
        BME280_FILTER_COEFF_16,
        BME280_NORMAL_MODE);

    if (ESP_OK != bme_rc)
    {
        ESP_LOGE("BME", "BME280 settings failed");
        while (1)
        {
            vTaskDelay(1);
        }
    }
#endif

    ESP_LOGI(pcTaskGetName(NULL), "Frequency is 433MHz");
    lora_set_frequency(433e6); // 433MHz

    lora_enable_crc();

    int cr = 1;
    int bw = 7;
    int sf = 7;

    lora_set_coding_rate(cr);
    ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

    lora_set_bandwidth(bw);

    ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

    lora_set_spreading_factor(sf);
    ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

    lora_dump_registers();

#if CONFIG_LORA_TRANSMITTER
    xTaskCreate(&task_tx, "TX", 1024 * 3, NULL, 5, NULL);
#endif
#if CONFIG_LORA_RECEIVER
    xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
#endif
}