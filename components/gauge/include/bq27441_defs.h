#ifndef _BQ27441_DEFS_H_
#define _BQ27441_DEFS_H_

/**
 * @file bq27441_defs.h
 * @brief Definitions for the BQ27441 fuel gauge.
 */

/**
 * @brief Sensor interface functions for BQ27441.
 */
extern sensor_interface_t bq27441_interface;

/**
 * @brief Configuration structure for BQ27441.
 */
typedef struct
{
    uint8_t i2c_address;
    uint16_t design_capacity;   // in mAh
    uint16_t terminate_voltage; // in mV
} bq27441_config_t;

/**
 * @brief 7-bit I2C address of the BQ27441 fuel gauge.
 */
#define BQ27441_ADDR CONFIG_BQ27441_I2C_ADDRESS

/**
 * @brief Offset for the design capacity register in subclass 82.
 */
#define BQ27441_DESIGN_CAPACITY_OFFSET 0x0A

/**
 * @brief Offset for the terminate voltage register in subclass 82.
 */
#define BQ27441_TERMINATE_VOLTAGE_OFFSET 0x0C

/**
 * @brief Offset for the design capacity read command.
 */
#define BQ27441_CMD_READ_DESIGN_CAPACITY 0x4A

/**
 * @brief Command to read the State of Charge (SoC).
 */
#define BQ27441_CMD_SOC 0x1C

/**
 * @brief Command to read the voltage.
 */
#define BQ27441_CMD_VOLTAGE 0x04

/**
 * @brief Data class register address.
 */
#define BQ27441_REG_CLASS_ID 0x3E

/**
 * @brief Data block control register address.
 */
#define BQ27441_REG_BLOCK_DATA_CONTROL 0x61

/**
 * @brief Data block offset register address.
 */
#define BQ27441_REG_BLOCK_OFFSET 0x3F

/**
 * @brief Data block start address.
 */
#define BQ27441_REG_BLOCK_DATA 0x40

/**
 * @brief Checksum register address.
 */
#define BQ27441_REG_CHECKSUM 0x60

/**
 * @brief Command for entering config mode (SET_CFGUPDATE).
 */
#define BQ27441_CMD_SET_CFGUPDATE 0x0013

/**
 * @brief Command for soft reset (SOFT_RESET).
 */
#define BQ27441_CMD_SOFT_RESET 0x0042

/**
 * @brief Command for exiting config mode (EXIT_CFGUPDATE).
 */
#define BQ27441_CMD_EXIT_CFGUPDATE 0x0043

/**
 * @brief Data class ID for state data.
 */
#define BQ27441_ID_STATE 82

/**
 * @brief Command to read flags.
 */
#define BQ27441_CMD_READ_FLAGS 0x06

/**
 * @brief Command register address for control commands.
 */
#define BQ27441_CMD_CONTROL 0x00

/**
 * @brief Unseal key for the BQ27441 device.
 */
#define BQ27441_UNSEAL_KEY0 0x14
#define BQ27441_UNSEAL_KEY1 0x04

#endif // _BQ27441_DEFS_H_
