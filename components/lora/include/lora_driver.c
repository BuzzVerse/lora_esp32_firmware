#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lora_driver_defs.h"

static spi_device_handle_t __spi;

static int __implicit;
static long __frequency;
static int __send_packet_lost = 0;

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
esp_err_t
lora_write_reg(int reg, int val)
{
   uint8_t out[2] = {0x80 | reg, val};
   uint8_t in[2];

   spi_transaction_t t = {
       .flags = 0,
       .length = 8 * sizeof(out),
       .tx_buffer = out,
       .rx_buffer = in};

   return spi_device_transmit(__spi, &t);
}

/**
 * Write a buffer to a register.
 * @param reg Register index.
 * @param val Value to write.
 * @param len Byte length to write.
 *
 * @return esp_err_t
 */
esp_err_t
lora_write_reg_buffer(int reg, uint8_t *val, int len)
{
   uint8_t *out;
   esp_err_t ret;

   out = (uint8_t *)malloc(len + 1);
   out[0] = 0x80 | reg;
   for (int i = 0; i < len; i++)
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
   return ret;
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 */
esp_err_t
lora_read_reg(int reg, uint8_t *val)
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
   return ret;
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 * @param len Byte length to read.
 */
esp_err_t
lora_read_reg_buffer(int reg, uint8_t *val, int len)
{
   uint8_t *out;
   uint8_t *in;
   esp_err_t ret;

   out = (uint8_t *)malloc(len + 1);
   in = (uint8_t *)malloc(len + 1);
   out[0] = reg;

   for (int i = 0; i < len; i++)
   {
      out[i + 1] = 0xff;
   }

   spi_transaction_t t = {
       .flags = 0,
       .length = 8 * (len + 1),
       .tx_buffer = out,
       .rx_buffer = in};

   ret = spi_device_transmit(__spi, &t);

   for (int i = 0; i < len; i++)
   {
      val[i] = in[i + 1];
   }

   free(out);
   free(in);

   return ret;
}

/**
 * Perform physical reset on the Lora chip
 */
void lora_reset(void)
{
   gpio_set_level(CONFIG_RST_GPIO, 0);
   vTaskDelay(pdMS_TO_TICKS(1));
   gpio_set_level(CONFIG_RST_GPIO, 1);
   vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
esp_err_t lora_explicit_header_mode(void)
{
   __implicit = 0;
   uint8_t reg_val;

   if (lora_read_reg(REG_MODEM_CONFIG_1, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   return lora_write_reg(REG_MODEM_CONFIG_1, reg_val & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
esp_err_t lora_implicit_header_mode(int size)
{
   esp_err_t ret;
   __implicit = 1;

   uint8_t reg_val;

   ret = lora_read_reg(REG_MODEM_CONFIG_1, &reg_val);

   ret += lora_write_reg(REG_MODEM_CONFIG_1, reg_val | 0x01);
   ret += lora_write_reg(REG_PAYLOAD_LENGTH, size);

   return ret;
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
esp_err_t lora_idle(void)
{
   return lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
esp_err_t lora_sleep(void)
{
   return lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
esp_err_t lora_receive(void)
{
   return lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
esp_err_t lora_set_tx_power(int level)
{
   // RF9x module uses PA_BOOST pin
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

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
esp_err_t lora_set_frequency(long frequency)
{
   esp_err_t ret;
   __frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   ret = lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   ret += lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   ret += lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));

   return ret;
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
esp_err_t lora_set_spreading_factor(int sf)
{
   esp_err_t ret;
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

/**
 * Get spreading factor.
 */
esp_err_t lora_get_spreading_factor(uint8_t *sf)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_2, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   *sf = (reg_val >> 4);
   return ESP_OK;
}

/**
 * Set Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 * @param mode mode of DIO(0 to 3)
 */
esp_err_t lora_set_dio_mapping(int dio, int mode)
{
   int _mode;
   esp_err_t ret;

   if (dio < 4)
   {
      ret = lora_read_reg(REG_DIO_MAPPING_1, &_mode);
      if (ret != ESP_OK)
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

   return ESP_FAIL;
}

/**
 * Get Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 */
esp_err_t lora_get_dio_mapping(int dio, int *mapping)
{
   int _mode;
   esp_err_t ret;

   if (dio < 4)
   {
      ret = lora_read_reg(REG_DIO_MAPPING_1, &_mode);
      if (ret != ESP_OK)
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

      return ESP_OK;
   }
   else if (dio < 6)
   {
      ret = lora_read_reg(REG_DIO_MAPPING_2, &_mode);
      if (ret != ESP_OK)
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

      return ESP_OK;
   }
   return ESP_FAIL;
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
esp_err_t lora_set_bandwidth(int sbw)
{
   uint8_t reg_val;

   if (lora_read_reg(REG_MODEM_CONFIG_1, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   if (sbw < 10)
   {
      return lora_write_reg(REG_MODEM_CONFIG_1, (reg_val & 0x0f) | (sbw << 4));
   }

   return ESP_FAIL;
}

/**
 * Get bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
esp_err_t lora_get_bandwidth(int *bw)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_1, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   *bw = ((reg_val & 0xf0) >> 4);
   return ESP_OK;
}

/**
 * Set coding rate
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
esp_err_t lora_set_coding_rate(int denominator)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_1, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   if (denominator < 5)
      denominator = 5;
   else if (denominator > 8)
      denominator = 8;

   int cr = denominator - 4;
   return lora_write_reg(REG_MODEM_CONFIG_1, (reg_val & 0xf1) | (cr << 1));
}

/**
 * Get coding rate
 */
esp_err_t lora_get_coding_rate(int *cr)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_1, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   *cr = (reg_val & 0x0e) >> 1;
   return ESP_OK;
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
esp_err_t lora_set_preamble_length(long length)
{
   esp_err_t ret;
   ret = lora_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   ret += lora_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
   return ret;
}

/**
 * Get the size of preamble.
 */
esp_err_t lora_get_preamble_length(long *preamble)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_PREAMBLE_MSB, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   *preamble = (reg_val << 8);

   if (lora_read_reg(REG_PREAMBLE_LSB, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   *preamble = *preamble + reg_val;

   return ESP_OK;
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
esp_err_t lora_set_sync_word(int sw)
{
   return lora_write_reg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
esp_err_t lora_enable_crc(void)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_2, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   return lora_write_reg(REG_MODEM_CONFIG_2, reg_val | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
esp_err_t lora_disable_crc(void)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_MODEM_CONFIG_2, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   return lora_write_reg(REG_MODEM_CONFIG_2, reg_val & 0xfb);
}

/**
 * Perform hardware initialization.
 */
esp_err_t lora_init(void)
{
   esp_err_t ret;

   /*
    * Configure CPU hardware to communicate with the radio chip
    */
   ret = gpio_reset_pin(CONFIG_RST_GPIO);
   ret += gpio_set_direction(CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);
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

   ret += spi_bus_initialize(HOST_ID, &bus, SPI_DMA_CH_AUTO);
   assert(ret == ESP_OK);

   spi_device_interface_config_t dev = {
       .clock_speed_hz = 9000000,
       .mode = 0,
       .spics_io_num = CONFIG_CS_GPIO,
       .queue_size = 7,
       .flags = 0,
       .pre_cb = NULL};
   ret = spi_bus_add_device(HOST_ID, &dev, &__spi);
   assert(ret == ESP_OK);

   /*
    * Perform hardware reset.
    */
   lora_reset();

   /*
    * Check version.
    */
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
      return 0; // Illegal version

   /*
    * Default configuration.
    */
   ret = lora_sleep();
   ret += lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
   ret += lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
   uint8_t lna_val;
   lora_read_reg(REG_LNA, &lna_val);
   ret += lora_write_reg(REG_LNA, lna_val | 0x03);
   ret += lora_write_reg(REG_MODEM_CONFIG_3, 0x04); // 00000100
   ret += lora_set_tx_power(17);

   ret += lora_idle();
   return ret;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
esp_err_t lora_send_packet(uint8_t *buf, int size)
{
   esp_err_t ret;
   /*
    * Transfer data to radio.
    */
   ret = lora_idle();
   ret += lora_write_reg(REG_FIFO_ADDR_PTR, 0);

   ret += lora_write_reg_buffer(REG_FIFO, buf, size);

   ret += lora_write_reg(REG_PAYLOAD_LENGTH, size);

   /*
    * Start transmission and wait for conclusion.
    */
   ret += lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

   assert(ret == ESP_OK);

   int loop = 0;
   while (1)
   {
      int irq;
      ret = lora_read_reg(REG_IRQ_FLAGS, &irq);
      assert(ret == ESP_OK);
      ESP_LOGD(TAG, "lora_read_reg=0x%x", irq);

      if ((irq & IRQ_TX_DONE_MASK) == IRQ_TX_DONE_MASK)
         break;
      loop++;
      if (loop == 10)
         break;
      vTaskDelay(2);
   }
   if (loop == 10)
   {
      __send_packet_lost++;
      ESP_LOGE(TAG, "lora_send_packet Fail");
   }

   return lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
esp_err_t lora_receive_packet(uint8_t *buf, int *return_len, int size)
{
   /*
    * Check interrupts.
    */
   int irq;
   int len = 0;
   esp_err_t ret;

   ret = lora_read_reg(REG_IRQ_FLAGS, &irq);
   ret += lora_write_reg(REG_IRQ_FLAGS, irq);
   assert(ret == ESP_OK);

   if ((irq & IRQ_RX_DONE_MASK) == 0)
      return ESP_FAIL;
   if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK)
      return ESP_FAIL;

   /*
    * Find packet size.
    */
   if (__implicit)
      lora_read_reg(REG_PAYLOAD_LENGTH, &len);
   else
      lora_read_reg(REG_RX_NB_BYTES, &len);

   /*
    * Transfer data from radio.
    */
   lora_idle();
   
   uint8_t reg_val;

   lora_read_reg(REG_FIFO_RX_CURRENT_ADDR, &reg_val);
   lora_write_reg(REG_FIFO_ADDR_PTR, reg_val);
   
   if (len > size)
      len = size;

   lora_read_reg_buffer(REG_FIFO, buf, len);

   *return_len = len;

   return ESP_OK;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
esp_err_t lora_received(bool *received)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_IRQ_FLAGS, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   if (reg_val & IRQ_RX_DONE_MASK)
      *received = true;
   else
      *received = false;

   return ESP_OK;
}

/**
 * Returns RegIrqFlags.
 */
esp_err_t lora_get_irq(int *irq_flags)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_IRQ_FLAGS, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   *irq_flags = reg_val;
   return ESP_OK;
}

/**
 * Return lost send packet count.
 */
int lora_packet_lost(void)
{
   return (__send_packet_lost);
}

/**
 * Return last packet's RSSI.
 */
esp_err_t lora_packet_rssi(int *rssi)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_PKT_RSSI_VALUE, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   *rssi = reg_val - (__frequency < 868E6 ? 164 : 157);
   return ESP_OK;
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
esp_err_t lora_packet_snr(int *snr)
{
   uint8_t reg_val;
   if (lora_read_reg(REG_PKT_SNR_VALUE, &reg_val) != ESP_OK)
   {
      return ESP_FAIL;
   }

   *snr = ((int8_t)reg_val) * 0.25;
   return ESP_OK;
}

/**
 * Shutdown hardware.
 */
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

esp_err_t lora_dump_registers(void)
{
   int i;
   printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   for (i = 0; i < 0x40; i++)
   {
      uint8_t reg_val;
      if (lora_read_reg(i, &reg_val) != ESP_OK)
      {
         return ESP_FAIL;
      }
      printf("%02X ", reg_val);
      if ((i & 0x0f) == 0x0f)
         printf("\n");
   }
   printf("\n");
   return ESP_OK;
}
