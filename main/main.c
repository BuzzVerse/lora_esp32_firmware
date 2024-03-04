#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lora_driver.h"

#define CONFIG_433MHZ 1
static uint8_t buf[3];
static uint8_t rx_buf[256];

#if CONFIG_LORA_TRANSMITTER
void task_tx(void *pvParameters)
{
    double temperature = 0, pressure = 0, humidity = 0;

    ESP_LOGI(pcTaskGetName(NULL), "Start");

    while (1)
    {
        bme280_read_temperature(&temperature);
        bme280_read_pressure(&pressure);
        bme280_read_humidity(&humidity);

        ESP_LOGI(pcTaskGetName(NULL), "Temperature: %.2f C", temperature);
        ESP_LOGI(pcTaskGetName(NULL), "Pressure: %.2f hPa", pressure);
        ESP_LOGI(pcTaskGetName(NULL), "Humidity: %.2f %%", humidity);

        buf[0] = temperature * 100;
        buf[1] = pressure / 100;
        buf[2] = humidity * 100;

        lora_send_packet(buf, 3);

        ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", 4);
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

    while (1)
    {
        lora_receive(); // put into receive mode
        bool hasReceived;
        lora_received(&hasReceived);

        if (hasReceived)
        {
            uint8_t rxLen = 0;
            lora_receive_packet(rx_buf, &rxLen, sizeof(rx_buf));
            ESP_LOGI(pcTaskGetName(NULL), "%u byte packet received:[ %u %u %u %u ]", rxLen, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
        }
        vTaskDelay(1); // Avoid WatchDog alerts
    }                  // end while
}
#endif

// Main application
void app_main(void)
{
    if (ESP_OK != lora_init() || ESP_OK != bme280_init_driver(CONFIG_BME280_I2C_ADDRESS))
    {
        ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
        while (1)
        {
            vTaskDelay(1);
        }
    }

    esp_err_t bme_rc = ESP_OK;

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
        ESP_LOGE(TAG_MAIN, "BME280 settings failed");
        while (1)
        {
            vTaskDelay(1);
        }
    }

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