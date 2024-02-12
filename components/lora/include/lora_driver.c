#include <string.h>
#include "lora_driver_defs.h"
#include "spi_api.h"

// to delete
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

typedef enum
{
   LORA_OK,
   LORA_FAIL,
   LORA_FAILED_SPI_WRITE,
   LORA_FAILED_SPI_WRITE_BUF,
   LORA_FAILED_SPI_READ,
   LORA_FAILED_SPI_READ_BUF,
   LORA_FAILED_SEND_PACKET,
} lora_status_t;

static uint8_t __implicit;
static long __frequency;
static uint8_t __send_packet_lost = 0;
static uint8_t *irq;

lora_status_t lora_write_reg(uint8_t reg, uint8_t val)
{
   spi_status_t status = spi_write(reg, val);
   if (SPI_OK == status)
   {
      return LORA_OK;
   }
   else
   {
      return LORA_FAILED_SPI_WRITE;
   }
}

lora_status_t lora_write_reg_buffer(uint8_t reg, uint8_t *val, uint8_t len)
{
   spi_status_t status = spi_write_buf(reg, val, len);
   if (SPI_OK == status)
   {
      return LORA_OK;
   }
   else
   {
      return LORA_FAILED_SPI_WRITE_BUF;
   }
}

lora_status_t lora_read_reg(uint8_t reg, uint8_t *val)
{
   spi_status_t status = spi_read(reg, val);
   if (SPI_OK == status)
   {
      return LORA_OK;
   }
   else
   {
      return LORA_FAIL;
   }
}

lora_status_t lora_read_reg_buffer(uint8_t reg, uint8_t *val, uint8_t len)
{
   spi_status_t status = spi_read_buf(reg, val, len);
   if (SPI_OK == status)
   {
      return LORA_OK;
   }
   else
   {
      return LORA_FAILED_SPI_READ_BUF;
   }
}

// lora_status_t gpio_set_level(uint8_t gpio_num, uint8_t level)
// {
// }

// lora_status_t vTaskDelay(uint8_t delay)
// {
// }

void lora_reset(void)
{
   gpio_set_level(CONFIG_RST_GPIO, 0);
   vTaskDelay(pdMS_TO_TICKS(1));
   gpio_set_level(CONFIG_RST_GPIO, 1);
   vTaskDelay(pdMS_TO_TICKS(10));
}

lora_status_t lora_explicit_header_mode(void)
{
   __implicit = 0;
   uint8_t reg_val;

   if (lora_read_reg(REG_MODEM_CONFIG_1, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   return lora_write_reg(REG_MODEM_CONFIG_1, reg_val & 0xfe);
}

lora_status_t lora_implicit_header_mode(uint8_t size)
{
   lora_status_t ret;
   __implicit = 1;

   uint8_t reg_val;

   ret = lora_read_reg(REG_MODEM_CONFIG_1, &reg_val);

   ret += lora_write_reg(REG_MODEM_CONFIG_1, reg_val | 0x01);
   ret += lora_write_reg(REG_PAYLOAD_LENGTH, size);

   return ret;
}

lora_status_t lora_idle(void)
{
   return lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

lora_status_t lora_sleep(void)
{
   return lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

lora_status_t lora_receive(void)
{
   return lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

lora_status_t lora_set_tx_power(uint8_t level)
{
   if (level < 2)
   {
      level = 2;
   }
   else if (level > 17)
   {
      level = 17;
   }

   return lora_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

lora_status_t lora_set_frequency(long frequency)
{
   lora_status_t ret;
   __frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   ret = lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   ret += lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   ret += lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));

   return ret;
}

lora_status_t lora_set_spreading_factor(uint8_t sf)
{
   lora_status_t ret;
   if (sf < 6)
   {
      sf = 6;
   }
   else if (sf > 12)
   {
      sf = 12;
   }

   if (sf == 6)
   {
      ret = lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
      ret += lora_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
   }
   else
   {
      ret = lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
      ret += lora_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
   }

   uint8_t reg_val;
   ret += lora_read_reg(REG_MODEM_CONFIG_2, &reg_val);
   ret += lora_write_reg(REG_MODEM_CONFIG_2, (reg_val & 0x0f) | ((sf << 4) & 0xf0));
   return ret;
}

lora_status_t lora_get_spreading_factor(uint8_t *sf)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_2, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   *sf = (reg_val >> 4);
   return LORA_OK;
}

lora_status_t lora_set_dio_mapping(uint8_t dio, uint8_t mode)
{
   uint8_t _mode;
   lora_status_t ret;

   if (dio < 4)
   {
      ret = lora_read_reg(REG_DIO_MAPPING_1, &_mode);
      if (ret != LORA_OK)
      {
         return ret;
      }

      if (dio == 0)
      {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      }
      else if (dio == 1)
      {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      }
      else if (dio == 2)
      {
         _mode = _mode & 0xF3;
         _mode = _mode | (mode << 2);
      }
      else if (dio == 3)
      {
         _mode = _mode & 0xFC;
         _mode = _mode | mode;
      }

      ret += lora_write_reg(REG_DIO_MAPPING_1, _mode);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
      return ret;
   }
   else if (dio < 6)
   {
      ret = lora_read_reg(REG_DIO_MAPPING_2, &mode);

      if (dio == 4)
      {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      }
      else if (dio == 5)
      {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      }

      ret += lora_write_reg(REG_DIO_MAPPING_2, _mode);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
      return ret;
   }

   return LORA_FAIL;
}

lora_status_t lora_get_dio_mapping(uint8_t dio, uint8_t *mapping)
{
   uint8_t _mode;
   lora_status_t ret;

   if (dio < 4)
   {
      ret = lora_read_reg(REG_DIO_MAPPING_1, &_mode);
      if (ret != LORA_OK)
      {
         return ret;
      }

      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);

      if (dio == 0)
      {
         *mapping = ((_mode >> 6) & 0x03);
      }
      else if (dio == 1)
      {
         *mapping = ((_mode >> 4) & 0x03);
      }
      else if (dio == 2)
      {
         *mapping = ((_mode >> 2) & 0x03);
      }
      else if (dio == 3)
      {
         *mapping = (_mode & 0x03);
      }

      return LORA_OK;
   }
   else if (dio < 6)
   {
      ret = lora_read_reg(REG_DIO_MAPPING_2, &_mode);
      if (ret != LORA_OK)
      {
         return ret;
      }

      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);

      if (dio == 4)
      {
         *mapping = ((_mode >> 6) & 0x03);
      }
      else if (dio == 5)
      {
         *mapping = ((_mode >> 4) & 0x03);
      }

      return LORA_OK;
   }
   return LORA_FAIL;
}

lora_status_t lora_set_bandwidth(uint8_t sbw)
{
   uint8_t reg_val;

   if (lora_read_reg(REG_MODEM_CONFIG_1, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   if (sbw < 10)
   {
      return lora_write_reg(REG_MODEM_CONFIG_1, (reg_val & 0x0f) | (sbw << 4));
   }

   return LORA_FAIL;
}

lora_status_t lora_get_bandwidth(uint8_t *bw)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_1, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   *bw = ((reg_val & 0xf0) >> 4);
   return LORA_OK;
}

lora_status_t lora_set_coding_rate(uint8_t denominator)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_1, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   if (denominator < 5)
      denominator = 5;
   else if (denominator > 8)
      denominator = 8;

   uint8_t cr = denominator - 4;
   return lora_write_reg(REG_MODEM_CONFIG_1, (reg_val & 0xf1) | (cr << 1));
}

lora_status_t lora_get_coding_rate(uint8_t *cr)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_1, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   *cr = (reg_val & 0x0e) >> 1;
   return LORA_OK;
}

lora_status_t lora_set_preamble_length(long length)
{
   lora_status_t ret;
   ret = lora_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   ret += lora_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
   return ret;
}

lora_status_t lora_get_preamble_length(long *preamble)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_PREAMBLE_MSB, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   *preamble = (reg_val << 8);

   if (lora_read_reg(REG_PREAMBLE_LSB, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   *preamble = *preamble + reg_val;

   return LORA_OK;
}

lora_status_t lora_set_sync_word(uint8_t sw)
{
   return lora_write_reg(REG_SYNC_WORD, sw);
}

lora_status_t lora_enable_crc(void)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_2, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   return lora_write_reg(REG_MODEM_CONFIG_2, reg_val | 0x04);
}

lora_status_t lora_disable_crc(void)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_2, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   return lora_write_reg(REG_MODEM_CONFIG_2, reg_val & 0xfb);
}

lora_status_t lora_dump_registers(void)
{
   uint8_t i;
   printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   for (i = 0; i < 0x40; i++)
   {
      uint8_t reg_val;
      if (lora_read_reg(i, &reg_val) != LORA_OK)
      {
         return LORA_FAIL;
      }
      printf("%02X ", reg_val);
      if ((i & 0x0f) == 0x0f)
         printf("\n");
   }
   printf("\n");
   return LORA_OK;
}

lora_status_t lora_init(void)
{
   lora_status_t ret;

   spi_init();
   lora_reset();

   uint8_t version;
   uint8_t i = 0;
   while (i++ < TIMEOUT_RESET)
   {
      lora_read_reg(REG_VERSION, &version);
      ESP_LOGD(TAG, "version=0x%02x", version);
      if (version == 0x12)
         break;
      vTaskDelay(2);
   }
   ESP_LOGD(TAG, "i=%d, TIMEOUT_RESET=%d", i, TIMEOUT_RESET);

   if (i == TIMEOUT_RESET + 1)
      return 0;

   ret = lora_sleep();
   ret += lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
   ret += lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
   uint8_t lna_val;
   lora_read_reg(REG_LNA, &lna_val);
   ret += lora_write_reg(REG_LNA, lna_val | 0x03);
   ret += lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
   ret += lora_set_tx_power(17);

   ret += lora_idle();

   return ret;
}

lora_status_t lora_send_packet(uint8_t *buf, uint8_t size)
{
   lora_status_t ret;

   ret = lora_idle();
   ret += lora_write_reg(REG_FIFO_ADDR_PTR, 0);

   ret += lora_write_reg_buffer(REG_FIFO, buf, size);

   ret += lora_write_reg(REG_PAYLOAD_LENGTH, size);

   ret += lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

   if (LORA_OK != ret)
   {
      ESP_LOGE(TAG, "LORA_FAILED_SEND_PACKET");
      return LORA_FAILED_SEND_PACKET;
   }

   uint8_t loop = 0;
   uint8_t tmp = 0;

   irq = &tmp;

   while (1)
   {
      ret = lora_read_reg(REG_IRQ_FLAGS, irq);
      ESP_LOGI(TAG, "lora_read_reg=0x%x", *irq);

      if ((*irq & IRQ_TX_DONE_MASK) == IRQ_TX_DONE_MASK)
      {
         ESP_LOGI(TAG, "IRQ_TX_DONE_MASK");
         ESP_LOGI(TAG, "Time taken(ms): %d", loop * 200);
         break;
      }
      loop++;
      if (loop == 2000)
         break;
      vTaskDelay(20);
   }
   if (loop == 200)
   {
      __send_packet_lost++;
      ESP_LOGE(TAG, "lora_send_packet Fail");
   }

   return lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

lora_status_t lora_receive_packet(uint8_t *buf, uint8_t *return_len, uint8_t size)
{
   uint8_t irq;
   uint8_t len = 0;
   lora_status_t ret;

   ret = lora_read_reg(REG_IRQ_FLAGS, &irq);
   ret += lora_write_reg(REG_IRQ_FLAGS, irq);

   if ((irq & IRQ_RX_DONE_MASK) == 0)
      return LORA_FAIL;
   if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK)
      return LORA_FAIL;

   if (__implicit)
      lora_read_reg(REG_PAYLOAD_LENGTH, &len);
   else
      lora_read_reg(REG_RX_NB_BYTES, &len);

   lora_idle();

   uint8_t reg_val;

   lora_read_reg(REG_FIFO_RX_CURRENT_ADDR, &reg_val);
   lora_write_reg(REG_FIFO_ADDR_PTR, reg_val);

   if (len > size)
      len = size;

   lora_read_reg_buffer(REG_FIFO, buf, len);

   *return_len = len;

   return LORA_OK;
}

lora_status_t lora_received(bool *received)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_IRQ_FLAGS, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   if (reg_val & IRQ_RX_DONE_MASK)
      *received = true;
   else
      *received = false;

   return LORA_OK;
}

lora_status_t lora_get_irq(uint8_t *irq_flags)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_IRQ_FLAGS, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   *irq_flags = reg_val;
   return LORA_OK;
}

uint8_t lora_packet_lost(void)
{
   return (__send_packet_lost);
}

lora_status_t lora_packet_rssi(uint8_t *rssi)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_PKT_RSSI_VALUE, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   *rssi = reg_val - (__frequency < 868E6 ? 164 : 157);
   return LORA_OK;
}

lora_status_t lora_packet_snr(uint8_t *snr)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_PKT_SNR_VALUE, &reg_val) != LORA_OK)
   {
      return LORA_FAIL;
   }

   *snr = ((uint8_t)reg_val) * 0.25;
   return LORA_OK;
}

void lora_close(void)
{
   lora_sleep();
   //   close(__spi);  FIXME: end hardware features after lora_close
   //   close(__cs);
   //   close(__rst);
   //   __spi = -1;
   //   __cs = -1;
   //   __rst = -1;
}
