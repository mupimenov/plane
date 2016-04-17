/*
 * lsm303c.h
 *
 *  Created on: Mar 5, 2016
 *      Author: mupimenov
 */

#ifndef SRC_DRIVERS_LSM303C_H_
#define SRC_DRIVERS_LSM303C_H_

#include <stdbool.h>
#include <stdint.h>

#include "accelerometer.h"
#include "magneto.h"
#include "gpio.h"
#include "spi_device.h"

struct lsm303c_accelerometer {
	struct accelerometer_driver interface;

	struct {
		uint32_t reg_selector;

		uint8_t reg1;
		uint8_t reg2;
		uint8_t reg3;
		uint8_t reg4;
		uint8_t reg5;
		uint8_t reg6;
		uint8_t fifo_ctrl_reg;
	} config;

	spi_device_t *dev;
	struct gpio_pin *cs;
};

// REG_SELECTOR
#define LSM303C_ACC_SELECT_REG1								((uint32_t)0x00000001)
#define LSM303C_ACC_SELECT_REG2								((uint32_t)0x00000002)
#define LSM303C_ACC_SELECT_REG3								((uint32_t)0x00000004)
#define LSM303C_ACC_SELECT_REG4								((uint32_t)0x00000008)
#define LSM303C_ACC_SELECT_REG5								((uint32_t)0x00000010)
#define LSM303C_ACC_SELECT_REG6								((uint32_t)0x00000020)
#define LSM303C_ACC_SELECT_FIFO_CTRL_REG					((uint32_t)0x00000040)

// REG1
#define LSM303C_ACC_HIGH_RESOLUTION_ENABLE      ((uint8_t)0x80)
#define LSM303C_ACC_DATA_RATE_OFF           	((uint8_t)0x00)  /*!< Output Data Rate powerdown */
#define LSM303C_ACC_DATA_RATE_10HZ             	((uint8_t)0x10)  /*!< Output Data Rate = 10 Hz */
#define LSM303C_ACC_DATA_RATE_50HZ             	((uint8_t)0x20)  /*!< Output Data Rate = 50 Hz */
#define LSM303C_ACC_DATA_RATE_100HZ            	((uint8_t)0x30)  /*!< Output Data Rate = 100 Hz */
#define LSM303C_ACC_DATA_RATE_400HZ            	((uint8_t)0x50)  /*!< Output Data Rate = 400 Hz */
#define LSM303C_ACC_DATA_RATE_800HZ            	((uint8_t)0x60)  /*!< Output Data Rate = 800 Hz */
#define LSM303C_ACC_CONTINUOUS_UPDATE   		((uint8_t)0x00) /*!< Continuos Update */
#define LSM303C_ACC_UPDATE_ONREAD       		((uint8_t)0x08) /*!< Single Update: output registers not updated until MSB and LSB reading */
#define LSM303C_ACC_X_ENABLE                	((uint8_t)0x01)
#define LSM303C_ACC_Y_ENABLE                	((uint8_t)0x02)
#define LSM303C_ACC_Z_ENABLE                	((uint8_t)0x04)
// or
#define LSM303C_ACC_AXES_ENABLE             	((uint8_t)0x07)

// REG2
#define LSM303C_ACC_CUTOFF_DATA_RATE_DIV50     	((uint8_t)0x00)
#define LSM303C_ACC_CUTOFF_DATA_RATE_DIV100    	((uint8_t)0x20)
#define LSM303C_ACC_CUTOFF_DATA_RATE_DIV9      	((uint8_t)0x40)
#define LSM303C_ACC_CUTOFF_DATA_RATE_DIV400     ((uint8_t)0x60)
#define LSM303C_ACC_HIGHPASS_MODE_REF_SIGNAL    ((uint8_t)0x08)
#define LSM303C_ACC_HIGHPASS_MODE_NORMAL        ((uint8_t)0x00)
#define LSM303C_ACC_HIGHPASS_TO_FIFO_ENABLE    	((uint8_t)0x04)

// REG3
#define LSM303C_ACC_FIFO_ENABLE        			((uint8_t)0x80)
#define LSM303C_ACC_FIFO_THRESHOLD_ENABLE       ((uint8_t)0x40)
#define LSM303C_ACC_INACTIVITY_INT_ENABLE       ((uint8_t)0x20)
#define LSM303C_ACC_IG2_INT_ENABLE       		((uint8_t)0x10)
#define LSM303C_ACC_IG1_INT_ENABLE       		((uint8_t)0x08)
#define LSM303C_ACC_FIFO_OVERRUN_INT_ENABLE     ((uint8_t)0x04)
#define LSM303C_ACC_FIFO_THRESHOLD_INT_ENABLE   ((uint8_t)0x02)
#define LSM303C_ACC_DATA_READY_INT_ENABLE   	((uint8_t)0x01)

// REG4
#define LSM303C_ACC_AA_FILTER_BANDWIDTH_400HZ  	((uint8_t)0x00)
#define LSM303C_ACC_AA_FILTER_BANDWIDTH_200HZ  	((uint8_t)0x40)
#define LSM303C_ACC_AA_FILTER_BANDWIDTH_100HZ  	((uint8_t)0x80)
#define LSM303C_ACC_AA_FILTER_BANDWIDTH_50HZ  	((uint8_t)0xC0)
#define LSM303C_ACC_FULLSCALE_2G            	((uint8_t)0x00)  /*!< ±2 g */
#define LSM303C_ACC_FULLSCALE_4G            	((uint8_t)0x20)  /*!< ±4 g */
#define LSM303C_ACC_FULLSCALE_8G            	((uint8_t)0x30)  /*!< ±8 g */
#define LSM303C_ACC_FULLSCALE_MASK            	((uint8_t)0x30)
#define LSM303C_ACC_BANDWIDTH_SCALE_SELECTION 	((uint8_t)0x08)
#define LSM303C_ACC_INC_ADDRESS_ON_RW 			((uint8_t)0x04)
#define LSM303C_ACC_DISABLE_I2C 				((uint8_t)0x02)
#define LSM303C_ACC_SPI_READ_WRITE_ENABLE		((uint8_t)0x01)

// REG5
#define LSM303C_ACC_DEBUG_ENABLE  				((uint8_t)0x80)
#define LSM303C_ACC_SOFT_RESET  				((uint8_t)0x40)
#define LSM303C_ACC_NO_DECIMATION  				((uint8_t)0x00)
#define LSM303C_ACC_DECIMATION_EVERY_2SAMPLES	((uint8_t)0x10)
#define LSM303C_ACC_DECIMATION_EVERY_4SAMPLES	((uint8_t)0x20)
#define LSM303C_ACC_DECIMATION_EVERY_8SAMPLES	((uint8_t)0x30)
#define LSM303C_ACC_INT_ACTIVE_HIGH				((uint8_t)0x00)
#define LSM303C_ACC_INT_ACTIVE_LOW				((uint8_t)0x02)
#define LSM303C_ACC_INT_PUSH_PULL				((uint8_t)0x00)
#define LSM303C_ACC_INT_OPEN_DRAIN				((uint8_t)0x01)

// REG6
#define LSM303C_ACC_REBOOT  					((uint8_t)0x80)

// FIFO_CTRL_REG
#define LSM303C_ACC_FIFO_BYPASS_MODE			((uint8_t)0x00)
#define LSM303C_ACC_FIFO_NORMAL_MODE			((uint8_t)0x20)
#define LSM303C_ACC_FIFO_STREAM_MODE			((uint8_t)0x40)
#define LSM303C_ACC_FIFO_STREAM_TO_FIFO_MODE	((uint8_t)0x60)
#define LSM303C_ACC_FIFO_BYPASS_TO_STREAM_MODE	((uint8_t)0x80)
#define LSM303C_ACC_FIFO_BYPASS_TO_NORMAL_MODE	((uint8_t)0xE0)

bool lsm303c_accelerometer_make(struct lsm303c_accelerometer *accel, spi_device_t *dev, struct gpio_pin *cs);

struct lsm303c_magneto {
	struct magneto_driver interface;

	struct {
		uint32_t reg_selector;

		uint8_t reg1;
		uint8_t reg2;
		uint8_t reg3;
		uint8_t reg4;
		uint8_t reg5;
	} config;

	spi_device_t *dev;
	struct gpio_pin *cs;
};

// REG_SELECTOR
#define LSM303C_MAG_SELECT_REG1								((uint32_t)0x00000001)
#define LSM303C_MAG_SELECT_REG2								((uint32_t)0x00000002)
#define LSM303C_MAG_SELECT_REG3								((uint32_t)0x00000004)
#define LSM303C_MAG_SELECT_REG4								((uint32_t)0x00000008)
#define LSM303C_MAG_SELECT_REG5								((uint32_t)0x00000010)

// REG1
#define LSM303C_MAG_TEMP_SENSOR_ENABLE				((uint8_t)0x80)
#define LSM303C_MAG_LOW_POWER_MODE					((uint8_t)0x00)
#define LSM303C_MAG_MEDIUM_PERFORMANCE_MODE			((uint8_t)0x20)
#define LSM303C_MAG_HIGH_PERFORMANCE_MODE			((uint8_t)0x40)
#define LSM303C_MAG_ULTRA_PERFORMANCE_MODE			((uint8_t)0x60)
#define LSM303C_MAG_DATA_RATE_0_625HZ				((uint8_t)0x00)
#define LSM303C_MAG_DATA_RATE_1_25HZ				((uint8_t)0x04)
#define LSM303C_MAG_DATA_RATE_2_5HZ					((uint8_t)0x08)
#define LSM303C_MAG_DATA_RATE_5HZ					((uint8_t)0x0C)
#define LSM303C_MAG_DATA_RATE_10HZ					((uint8_t)0x10)
#define LSM303C_MAG_DATA_RATE_20HZ					((uint8_t)0x14)
#define LSM303C_MAG_DATA_RATE_40HZ					((uint8_t)0x18)
#define LSM303C_MAG_DATA_RATE_80HZ					((uint8_t)0x1C)
#define LSM303C_MAG_SELF_TEST_ENABLE				((uint8_t)0x01)

// REG2
#define LSM303C_MAG_FULL_SCALE_DISABLE				((uint8_t)0x00)
#define LSM303C_MAG_FULL_SCALE_16GAUSS				((uint8_t)0x60)
#define LSM303C_MAG_REBOOT_MEMORY					((uint8_t)0x08)
#define LSM303C_MAG_SOFT_RESET						((uint8_t)0x04)

// REG3
#define LSM303C_MAG_DISABLE_I2C						((uint8_t)0x80)
#define LSM303C_MAG_LOW_POWER_ENABLE				((uint8_t)0x20)
#define LSM303C_MAG_SPI_READ_WRITE_ENABLE			((uint8_t)0x04)
#define LSM303C_MAG_CONTINUOUS_MODE					((uint8_t)0x00)
#define LSM303C_MAG_SINGLE_MODE						((uint8_t)0x01)
#define LSM303C_MAG_POWER_DOWN_MODE					((uint8_t)0x03)

// REG4
#define LSM303C_MAG_Z_AXIS_LOW_POWER_MODE			((uint8_t)0x00)
#define LSM303C_MAG_Z_AXIS_MEDIUM_PERFORMANCE_MODE	((uint8_t)0x04)
#define LSM303C_MAG_Z_AXIS_HIGH_PERFORMANCE_MODE	((uint8_t)0x08)
#define LSM303C_MAG_Z_AXIS_ULTRA_PERFORMANCE_MODE	((uint8_t)0x0C)
#define LSM303C_MAG_ENDIAN_MSB						((uint8_t)0x02)

// REG5
#define LSM303C_MAG_CONTINUOUS_UPDATE				((uint8_t)0x00)
#define LSM303C_MAG_UPDATE_ON_READ					((uint8_t)0x40)

bool lsm303c_magneto_make(struct lsm303c_magneto *mag, spi_device_t *dev, struct gpio_pin *cs);

#endif /* SRC_DRIVERS_LSM303C_H_ */
