#include "i2c.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL_IO // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA_IO // GPIO number for I2C master data
#define I2C_MASTER_NUM I2C_NUM_0                   // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 100000                  // I2C master clock frequency (100kHz)
#define I2C_MASTER_TX_BUF_DISABLE 0                // I2C master do not need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0                // I2C master do not need buffer
#define ACK_CHECK_EN 0x1                           // I2C master will check ack from slave

static const char *TAG = "I2C_MASTER_CUSTOM";

void delay_ms(uint32_t ticks)
{
    vTaskDelay(ticks / portTICK_PERIOD_MS);
}

esp_err_t i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len)
{

    ESP_LOGD(TAG, "Writing to 0x%x", reg_addr);
#if CONFIG_LOG_DEFAULT_LEVEL_DEBUG || CONFIG_LOG_DEFAULT_LEVEL_VERBOSE

    for (int i = 0; i < data_len; i++)
    {
        printf("0x%x ", data[i]);
    }
    printf("\n");

#endif

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len)
{

    ESP_LOGD(TAG, "Readinreadg from 0x%x", reg_addr);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (data_len > 1)
    {
        i2c_master_read(cmd, data, data_len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + data_len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

#if CONFIG_LOG_DEFAULT_LEVEL_DEBUG || CONFIG_LOG_DEFAULT_LEVEL_VERBOSE
    for (int i = 0; i < data_len; i++)
    {
        printf("0x%x ", data[i]);
    }
    printf("\n");
#endif

    return err;
}
