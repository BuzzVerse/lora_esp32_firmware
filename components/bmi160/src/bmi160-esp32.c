
// (License goes here)
// Also change this component to be managed instead.

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/*********************************************************************/
/* ESP32-IDF header files */
/*********************************************************************/
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "driver/i2c_master.h"
#include "bmi160.h"

/*********************************************************************/
/* Macro definitions and config variables */
/*********************************************************************/
#define	SCL_PIN		CONFIG_BMI160_I2C_SCL
#define SDA_PIN		CONFIG_BMI160_I2C_SDA

//Device address
#define BMI160_DEV_ADDR		BMI160_I2C_ADDR // 0x68, 0x69 if jumpered



/*********************************************************************/
/* global variables */
/*********************************************************************/

/*! @brief Device struct */
struct bmi160_dev bmi160dev;

/*! @brief variable to hold the bmi160 accel data */
struct bmi160_sensor_data bmi160_accel;

/*! @brief variable to hold the bmi160 gyro data */
struct bmi160_sensor_data bmi160_gyro;


/*********************************************************************/
/* static function declarations */
/*********************************************************************/

/*!
 * @brief   Mock-up function, SDO is not soldered yet.
 */
static void init_sensor_interface(void);

/*!
 * @brief   This internal API is used to initialize the bmi160 with low-power config.
 */
static void init_bmi160(void);

/*!
 * @brief   This internal API is used to initialize the sensor driver interface.
 * Stripped to I2C.
 */
static void init_bmi160_sensor_driver_interface(void);


/*********************************************************************/
/* function declarations */
/*********************************************************************/

bmi160_i2c_write();

/********************************************************************/
/* Device setup */
/********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi160 sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return esp_err_t
 *
 */
static esp_err_t init_bmi160(void)
{
    int8_t rslt;

    init_bmi160_sensor_driver_interface();
    rslt = bmi160_init(&bmi160dev);

    if (rslt == BMI160_OK)
    {
        printf("BMI160 initialization success !\n");
        printf("Chip ID 0x%X\n", bmi160dev.chip_id);
    }
    else
    {
        printf("BMI160 initialization failure !\n");
	return ESP_ERR;
    }

    // TODO:ZMIENIĆ TO I PODLINKOWAĆ POD LOW-POWER MODE
    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi160dev);
}

/*!
 *  @brief This internal API is used to set the sensor driver interface
 *  read/write data. The code has been truncated from the SPI setup
 *  and uses only I2C instead.
 * 
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi160_sensor_driver_interface(void)
{
    /* I2C setup */

    /* link read/write/delay function of host system to appropriate
     * bmi160 function call prototypes */
    bmi160dev.write = coines_write_i2c; //TODO:Replace this
    bmi160dev.read = coines_read_i2c; //TODO:Replace this
    bmi160dev.delay_ms = coines_delay_msec; //TODO:Replace this uint32_t delay_ms, 400/450 µs in low-power


    // The coines API internally looks like this:
    //int8_t coines_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);
    /*!
     *  @brief This API is used to write 8-bit register data on the I2C device.
     *
     *  @param[in] dev_addr : Device address for I2C write.
     *  @param[in] reg_addr : Starting address for writing the data.
     *  @param[in] reg_data : Data to be written.
     *  @param[in] count    : Number of bytes to write.
     *
     *  @return Results of API execution status.
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */

    /* set correct i2c address */
    bmi160dev.id = BMI160_DEV_ADDR;
    bmi160dev.intf = BMI160_I2C_INTF;

}

/*! @brief: i2c init */
/*esp_err_t i2c_master_init(void){
	
	i2c_master_bus_config_t i2c_mst_config = {
	    .clk_source = I2C_CLK_SRC_DEFAULT, // 100 KHz set this to hz
	    // TODO:CHANGE THIS TO REDUCE POWER CONSUMPTION
	    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html#power-management
	    .i2c_port = -1, //autoselect the i2c controller
	    .scl_io_num = SCL_PIN,
	    .sda_io_num = SDA_PIN,
	    .glitch_ignore_cnt = 7,
	    .flags.enable_internal_pullup = true,
	};
	
	.i2c_master_bus_handle_t bus_handler;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
	
	i2c_device_config_t dev_cfg = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address = 0x58,
	    .scl_speed_hz = 400000, //
	};
	
	//adds device 
	.i2c_master_bus_handle_t bus_handler;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}
*/

esp_err_t i2c_bmi160_write(){



}

