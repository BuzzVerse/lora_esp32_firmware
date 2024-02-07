/**
 * @file lora_driver.h
 * @brief LoRa driver interface for ESP32
 *
 * Provides an interface to control a LoRa device using ESP32. Includes initialization,
 * configuration, and data transmission/reception functionalities.
 */

#ifndef LORA_DRIVER_H
#define LORA_DRIVER_H

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lora_driver_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Handle for the SPI device connection to the LoRa module. */
extern spi_device_handle_t __spi;

/** @brief Implicit header mode indicator. */
extern int __implicit;
/** @brief Frequency of the LoRa module. */
extern long __frequency;
/** @brief Counter for lost packets during send operation. */
extern int __send_packet_lost;

/**
 * @brief Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 * @return esp_err_t Result of the SPI write operation.
 */
esp_err_t lora_write_reg(int reg, int val);

/**
 * @brief Write a buffer to a register.
 * @param reg Register index.
 * @param val Pointer to the buffer to write.
 * @param len Byte length to write.
 * @return esp_err_t Result of the SPI write operation.
 */
esp_err_t lora_write_reg_buffer(int reg, uint8_t *val, int len);

/**
 * @brief Read the current value of a register.
 * @param reg Register index.
 * @param val Pointer to store the read value.
 * @return esp_err_t Result of the SPI read operation.
 */
esp_err_t lora_read_reg(int reg, uint8_t *val);

/**
 * @brief Read the current value of a register into a buffer.
 * @param reg Register index.
 * @param val Buffer to store the read value(s).
 * @param len Byte length to read.
 * @return esp_err_t Result of the SPI read operation.
 */
esp_err_t lora_read_reg_buffer(int reg, uint8_t *val, int len);

/**
 * @brief Perform physical reset on the LoRa chip.
 */
void lora_reset(void);

/**
 * @brief Configure explicit header mode.
 */
esp_err_t lora_explicit_header_mode(void);

/**
 * @brief Configure implicit header mode.
 * @param size Size of the packets.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_implicit_header_mode(int size);

/**
 * @brief Sets the radio transceiver in idle mode.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_idle(void);

/**
 * @brief Sets the radio transceiver in sleep mode.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_sleep(void);

/**
 * @brief Sets the radio transceiver in receive mode.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_receive(void);

/**
 * @brief Configure power level for transmission.
 * @param level Power level (2-17, from least to most power).
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_set_tx_power(int level);

/**
 * @brief Set carrier frequency.
 * @param frequency Frequency in Hz.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_set_frequency(long frequency);

/**
 * @brief Set spreading factor.
 * @param sf Spreading factor (6-12).
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_set_spreading_factor(int sf);

/**
 * @brief Get spreading factor.
 * @param sf Pointer to store the spreading factor.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_get_spreading_factor(uint8_t *sf);

/**
 * @brief Set mapping of pins DIO0 to DIO5.
 * @param dio Number of DIO (0 to 5).
 * @param mode Mode of DIO (0 to 3).
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_set_dio_mapping(int dio, int mode);

/**
 * @brief Get mapping of pins DIO0 to DIO5.
 * @param dio Number of DIO (0 to 5).
 * @param mapping Pointer to store mapping mode.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_get_dio_mapping(int dio, int *mapping);

/**
 * @brief Set bandwidth (bit rate).
 * @param sbw Signal bandwidth (0 to 9).
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_set_bandwidth(int sbw);

/**
 * @brief Get bandwidth (bit rate).
 * @param bw Pointer to store bandwidth.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_get_bandwidth(int *bw);

/**
 * @brief Set coding rate.
 * @param denominator Denominator for the coding rate 4/x.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_set_coding_rate(int denominator);

/**
 * @brief Get coding rate.
 * @param cr Pointer to store coding rate.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_get_coding_rate(int *cr);

/**
 * @brief Set the size of preamble.
 * @param length Preamble length in symbols.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_set_preamble_length(long length);

/**
 * @brief Get the size of preamble.
 * @param preamble Pointer to store preamble length.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_get_preamble_length(long *preamble);

/**
 * @brief Change radio sync word.
 * @param sw New sync word to use.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_set_sync_word(int sw);

/**
 * @brief Enable appending/verifying packet CRC.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_enable_crc(void);

/**
 * @brief Disable appending/verifying packet CRC.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_disable_crc(void);

/**
 * @brief Perform hardware initialization.
 * @return esp_err_t Result of initialization.
 */
esp_err_t lora_init(void);

/**
 * @brief Send a packet.
 * @param buf Data to be sent.
 * @param size Size of data.
 * @return esp_err_t Result of send operation.
 */
esp_err_t lora_send_packet(uint8_t *buf, int size);

/**
 * @brief Read a received packet.
 * @param buf Buffer for the data.
 * @param len Pointer to store the number of bytes received.
 * @param size Available size in buffer (bytes).
 * @param packet Pointer to store packet status.
 * @return esp_err_t Result of receive operation.
 */
esp_err_t lora_receive_packet(uint8_t *buf, int *len, int size);

/**
 * @brief Check if there is data to read (packet received).
 * @param received Pointer to store the received status.
 * @return esp_err_t Result of check.
 */
esp_err_t lora_received(bool *received);

/**
 * @brief Returns RegIrqFlags.
 * @param irq_flags Pointer to store IRQ flags.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_get_irq(int *irq_flags);

/**
 * @brief Return lost send packet count.
 * @return Number of lost packets.
 */
int lora_packet_lost(void);

/**
 * @brief Return last packet's RSSI.
 * @param rssi Pointer to store RSSI value.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_packet_rssi(int *rssi);

/**
 * @brief Return last packet's SNR (signal to noise ratio).
 * @param snr Pointer to store SNR value.
 * @return esp_err_t Result of operation.
 */
esp_err_t lora_packet_snr(int *snr);

/**
 * @brief Shutdown hardware.
 */
void lora_close(void);

/**
 * @brief Dump LoRa registers for debugging.
 * @return esp_err_t Result of dump operation.
 */
esp_err_t lora_dump_registers(void);

#ifdef __cplusplus
}
#endif

#endif // LORA_DRIVER_H
