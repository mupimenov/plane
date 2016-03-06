/*
 * lsm303.c
 *
 *  Created on: Mar 5, 2016
 *      Author: mupimenov
 */

#include "lsm303c.h"

#include <string.h>

/*
 *
 *
 * Acceleration Registers
 *
 *
 **/

#define LSM303C_WHO_AM_I_ADDR_A           	0x0F  /* device identification register */
#define LSM303C_ACT_THS_ADDR_A              0x1E
#define LSM303C_ACT_DUR_ADDR_A              0x1F
#define LSM303C_CTRL_REG1_ADDR_A            0x20  /* Control register 1 acceleration */
#define LSM303C_CTRL_REG2_ADDR_A            0x21  /* Control register 2 acceleration */
#define LSM303C_CTRL_REG3_ADDR_A            0x22  /* Control register 3 acceleration */
#define LSM303C_CTRL_REG4_ADDR_A            0x23  /* Control register 4 acceleration */
#define LSM303C_CTRL_REG5_ADDR_A            0x24  /* Control register 5 acceleration */
#define LSM303C_CTRL_REG6_ADDR_A            0x25  /* Control register 6 acceleration */
#define LSM303C_CTRL_REG7_ADDR_A            0x26  /* Control register 6 acceleration */
#define LSM303C_STATUS_REG_ADDR_A           0x27  /* Status register acceleration */
#define LSM303C_OUT_X_L_ADDR_A              0x28  /* Output Register X acceleration */
#define LSM303C_OUT_X_H_ADDR_A              0x29  /* Output Register X acceleration */
#define LSM303C_OUT_Y_L_ADDR_A              0x2A  /* Output Register Y acceleration */
#define LSM303C_OUT_Y_H_ADDR_A              0x2B  /* Output Register Y acceleration */
#define LSM303C_OUT_Z_L_ADDR_A              0x2C  /* Output Register Z acceleration */
#define LSM303C_OUT_Z_H_ADDR_A              0x2D  /* Output Register Z acceleration */
#define LSM303C_FIFO_CTRL_ADDR_A            0x2E  /* Fifo control Register acceleration */
#define LSM303C_FIFO_SRC_ADDR_A             0x2F  /* Fifo src Register acceleration */

#define LSM303C_IG_CFG1_ADDR_A                 0x30  /* Interrupt 1 configuration Register acceleration */
#define LSM303C_IG_SRC1_ADDR_A                 0x31  /* Interrupt 1 source Register acceleration */
#define LSM303C_IG_THS_X1_ADDR_A               0x32
#define LSM303C_IG_THS_Y1_ADDR_A               0x33
#define LSM303C_IG_THS_Z1_ADDR_A               0x34

#define LSM303C_IG_DUR1_ADDR_A                 0x32
#define LSM303C_INT1_DURATION_ADDR_A           0x33  /* Interrupt 1 DURATION register acceleration */

#define LSM303C_INT2_CFG_ADDR_A                0x34  /* Interrupt 2 configuration Register acceleration */
#define LSM303C_INT2_SOURCE_ADDR_A             0x35  /* Interrupt 2 source Register acceleration */
#define LSM303C_INT2_THS_ADDR_A                0x36  /* Interrupt 2 Threshold register acceleration */
#define LSM303C_INT2_DURATION_ADDR_A           0x37  /* Interrupt 2 DURATION register acceleration */

#define LSM303C_CLICK_CFG_ADDR_A               0x38  /* Click configuration Register acceleration */
#define LSM303C_CLICK_SOURCE_ADDR_A            0x39  /* Click 2 source Register acceleration */
#define LSM303C_CLICK_THS_ADDR_A               0x3A  /* Click 2 Threshold register acceleration */

#define LSM303C_TIME_LIMIT_ADDR_A              0x3B  /* Time Limit Register acceleration */
#define LSM303C_TIME_LATENCY_ADDR_A            0x3C  /* Time Latency Register acceleration */
#define LSM303C_TIME_WINDOW_ADDR_A             0x3D  /* Time window register acceleration */

// STATUS_REG
#define LSM303C_ACC_ZYX_OVERRUN 					0x80
#define LSM303C_ACC_Z_OVERRUN 						0x40
#define LSM303C_ACC_Y_OVERRUN 						0x20
#define LSM303C_ACC_X_OVERRUN 						0x10
#define LSM303C_ACC_ZYX_NEWDATA 					0x08
#define LSM303C_ACC_Z_NEWDATA 						0x04
#define LSM303C_ACC_Y_NEWDATA 						0x02
#define LSM303C_ACC_X_NEWDATA 						0x01

/*
 *
 *
 * Magnetic field Registers
 *
 *
 **/

#define LSM303C_CTRL_REG1_M               0x20  /* Magnetic control register 1 */
#define LSM303C_CTRL_REG2_M               0x21  /* Magnetic control register 2 */
#define LSM303C_CTRL_REG3_M               0x22  /* Magnetic control register 3 */
#define LSM303C_CTRL_REG4_M               0x23  /* Magnetic control register 4 */
#define LSM303C_CTRL_REG5_M               0x24  /* Magnetic control register 5 */

#define LSM303C_STATUS_REG_M              0x27  /* Magnetic status register M  */

#define LSM303C_OUT_X_L_M                 0x28  /* Output Register X magnetic field */
#define LSM303C_OUT_X_H_M                 0x29  /* Output Register X magnetic field */
#define LSM303C_OUT_Y_L_M                 0x2A  /* Output Register Y magnetic field */
#define LSM303C_OUT_Y_H_M                 0x2B  /* Output Register Y magnetic field */
#define LSM303C_OUT_Z_L_M                 0x2C  /* Output Register Z magnetic field */
#define LSM303C_OUT_Z_H_M                 0x2D  /* Output Register Z magnetic field */

#define LSM303C_TEMP_OUT_L_M              0x2E  /* Temperature Register magnetic field */
#define LSM303C_TEMP_OUT_H_M              0x2F  /* Temperature Register magnetic field */

#define LSM303C_INT_CFG_M                 0x30  /* Axis interrupt configuration        */
#define LSM303C_INT_SRC_M                 0x31  /* Axis interrupt source               */
#define LSM303C_INT_THS_L_M               0x32  /* Interrupt threshold L               */
#define LSM303C_INT_THS_H_M               0x33  /* Interrupt threshold M               */

#define LMS303C_ACC_ID                   	((uint8_t)0x41)
#define LMS303C_MAG_ID                      ((uint8_t)0x3D)

#define LSM303C_ACC_SENSITIVITY_2G     	((float)0.061f)  /*!< accelerometer sensitivity with 2 g full scale [mg/LSB] */
#define LSM303C_ACC_SENSITIVITY_4G     	((float)0.122f)  /*!< accelerometer sensitivity with 4 g full scale [mg/LSB] */
#define LSM303C_ACC_SENSITIVITY_8G     	((float)0.244f)  /*!< accelerometer sensitivity with 8 g full scale [mg/LSB] */

#define LSM303C_ACC_HPI2S_INT1_DISABLE  ((uint8_t)0x00)
#define LSM303C_ACC_HPI2S_INT1_ENABLE	((uint8_t)0x01)
#define LSM303C_ACC_HPI2S_INT2_DISABLE  ((uint8_t)0x00)
#define LSM303C_ACC_HPI2S_INT2_ENABLE   ((uint8_t)0x02)

#define LSM303C_IT1_CLICK               ((uint8_t)0x80)
#define LSM303C_IT1_AOI1                ((uint8_t)0x40)
#define LSM303C_IT1_AOI2                ((uint8_t)0x20)
#define LSM303C_IT1_DRY1                ((uint8_t)0x10)
#define LSM303C_IT1_DRY2                ((uint8_t)0x08)
#define LSM303C_IT1_WTM                 ((uint8_t)0x04)
#define LSM303C_IT1_OVERRUN             ((uint8_t)0x02)

#define LSM303C_IT2_CLICK               ((uint8_t)0x80)
#define LSM303C_IT2_INT1                ((uint8_t)0x40)
#define LSM303C_IT2_INT2                ((uint8_t)0x20)
#define LSM303C_IT2_BOOT                ((uint8_t)0x10)
#define LSM303C_IT2_ACT                 ((uint8_t)0x08)
#define LSM303C_IT2_HLACTIVE            ((uint8_t)0x02)

#define LSM303C_OR_COMBINATION          ((uint8_t)0x00)  /*!< OR combination of enabled IRQs */
#define LSM303C_AND_COMBINATION         ((uint8_t)0x80)  /*!< AND combination of enabled IRQs */
#define LSM303C_MOV_RECOGNITION         ((uint8_t)0x40)  /*!< 6D movement recognition */
#define LSM303C_POS_RECOGNITION         ((uint8_t)0xC0)  /*!< 6D position recognition */

#define LSM303C_Z_HIGH                  ((uint8_t)0x20)  /*!< Z High enabled IRQs */
#define LSM303C_Z_LOW                   ((uint8_t)0x10)  /*!< Z low enabled IRQs */
#define LSM303C_Y_HIGH                  ((uint8_t)0x08)  /*!< Y High enabled IRQs */
#define LSM303C_Y_LOW                   ((uint8_t)0x04)  /*!< Y low enabled IRQs */
#define LSM303C_X_HIGH                  ((uint8_t)0x02)  /*!< X High enabled IRQs */
#define LSM303C_X_LOW                   ((uint8_t)0x01)  /*!< X low enabled IRQs */

#define LSM303C_Z_DOUBLE_CLICK          ((uint8_t)0x20)  /*!< Z double click IRQs */
#define LSM303C_Z_SINGLE_CLICK          ((uint8_t)0x10)  /*!< Z single click IRQs */
#define LSM303C_Y_DOUBLE_CLICK          ((uint8_t)0x08)  /*!< Y double click IRQs */
#define LSM303C_Y_SINGLE_CLICK          ((uint8_t)0x04)  /*!< Y single click IRQs */
#define LSM303C_X_DOUBLE_CLICK          ((uint8_t)0x02)  /*!< X double click IRQs */
#define LSM303C_X_SINGLE_CLICK          ((uint8_t)0x01)  /*!< X single click IRQs */

#define LSM303C_INT1INTERRUPT_DISABLE   ((uint8_t)0x00)
#define LSM303C_INT1INTERRUPT_ENABLE    ((uint8_t)0x80)

#define LSM303C_INT1INTERRUPT_LOW_EDGE  ((uint8_t)0x20)
#define LSM303C_INT1INTERRUPT_HIGH_EDGE ((uint8_t)0x00)

static bool _accel_io_init(struct lsm303c_accelerometer *accel)
{
	do
	{
		if (!gpio_configure(accel->cs))
			break;
		gpio_set(accel->cs);

		if (!device_open(accel->dev))
			break;

		device_setup(accel->dev, SPI_DEVICE_PARAMETER_CLOCK, 1000000UL);
		device_setup(accel->dev, SPI_DEVICE_PARAMETER_BITS, SPI_8_BIT);
		device_setup(accel->dev, SPI_DEVICE_PARAMETER_PHASE, SPI_PHASE_0);
		device_setup(accel->dev, SPI_DEVICE_PARAMETER_POLARITY, SPI_POLARITY_0);
		device_setup(accel->dev, SPI_DEVICE_PARAMETER_MODE, SPI_MODE_MASTER);
		device_setup(accel->dev, SPI_DEVICE_PARAMETER_DIRECTION, SPI_DIRECTION_HALF_DUPLEX_TX);

		if (!device_configure(accel->dev))
			break;

		return true;
	} while (0);

	return false;
}

static bool _accel_io_deinit(struct lsm303c_accelerometer *accel)
{
	return device_close(accel->dev);
}

static void _accel_io_write(struct lsm303c_accelerometer *accel, uint8_t address, const uint8_t *data, uint16_t size)
{
	/* Set chip select Low at the start of the transmission */
	gpio_clear(accel->cs);

	device_setup(accel->dev, SPI_DEVICE_PARAMETER_DIRECTION, SPI_DIRECTION_HALF_DUPLEX_TX);
	device_configure(accel->dev);

	/* Send the Address of the indexed register */
	device_write(accel->dev, 0, &address, 1);

	/* Send the data that will be written into the device (MSB First) */
	device_write(accel->dev, 0, data, size);

	/* Set chip select High at the end of the transmission */
	gpio_set(accel->cs);
}

static void _accel_io_read(struct lsm303c_accelerometer *accel, uint8_t address, uint8_t *data, uint16_t size)
{
	address |= (uint8_t)(0x80);

	/* Set chip select Low at the start of the transmission */
	gpio_clear(accel->cs);

	device_setup(accel->dev, SPI_DEVICE_PARAMETER_DIRECTION, SPI_DIRECTION_HALF_DUPLEX_TX);
	device_configure(accel->dev);

	/* Send the Address of the indexed register */
	device_write(accel->dev, 0, &address, 1);

	device_setup(accel->dev, SPI_DEVICE_PARAMETER_DIRECTION, SPI_DIRECTION_HALF_DUPLEX_RX);
	device_configure(accel->dev);

	/* Receive the data that will be read from the device (MSB First) */
	device_read(accel->dev, 0, data, size);

	/* Set chip select High at the end of the transmission */
	gpio_set(accel->cs);
}

static bool _lsm303c_open(struct accelerometer_driver *__a)
{
	struct lsm303c_accelerometer *accel = (struct lsm303c_accelerometer *)__a;

	do
	{
		if (!_accel_io_init(accel))
			break;

		return true;
	} while (0);

	return false;
}

static bool _lsm303c_close(struct accelerometer_driver *__a)
{
	struct lsm303c_accelerometer *accel = (struct lsm303c_accelerometer *)__a;

	do
	{
		if (!_accel_io_deinit(accel))
			break;

		return true;
	} while (0);

	return false;
}

static bool _lsm303c_configure(struct accelerometer_driver *__a)
{
	struct lsm303c_accelerometer *accel = (struct lsm303c_accelerometer *)__a;

	do
	{
		uint8_t tmp = LSM303C_ACC_SPI_READ_WRITE_ENABLE;

		_accel_io_write(accel, LSM303C_CTRL_REG4_ADDR_A, &tmp, 1);
		_accel_io_read(accel, LSM303C_WHO_AM_I_ADDR_A, &tmp, 1);
		if (tmp != LMS303C_ACC_ID)
			break;

		if (accel->config.reg_selector & LSM303C_SELECT_REG1)
		{
			_accel_io_write(accel, LSM303C_CTRL_REG1_ADDR_A, &accel->config.reg1, 1);
		}

		if (accel->config.reg_selector & LSM303C_SELECT_REG2)
		{
			_accel_io_write(accel, LSM303C_CTRL_REG2_ADDR_A, &accel->config.reg2, 1);
		}

		if (accel->config.reg_selector & LSM303C_SELECT_REG3)
		{
			_accel_io_write(accel, LSM303C_CTRL_REG3_ADDR_A, &accel->config.reg3, 1);
		}

		if (accel->config.reg_selector & LSM303C_SELECT_REG4)
		{
			_accel_io_write(accel, LSM303C_CTRL_REG4_ADDR_A, &accel->config.reg4, 1);
		}

		if (accel->config.reg_selector & LSM303C_SELECT_REG5)
		{
			_accel_io_write(accel, LSM303C_CTRL_REG5_ADDR_A, &accel->config.reg5, 1);
		}

		if (accel->config.reg_selector & LSM303C_SELECT_REG6)
		{
			_accel_io_write(accel, LSM303C_CTRL_REG6_ADDR_A, &accel->config.reg6, 1);
		}

		if (accel->config.reg_selector & LSM303C_SELECT_FIFO_CTRL_REG) {
			_accel_io_write(accel, LSM303C_FIFO_CTRL_ADDR_A, &accel->config.fifo_ctrl_reg, 1);
		}

		return true;
	} while (0);

	return false;
}

static bool _wait_data_ready(struct lsm303c_accelerometer *accel, bool x, bool y, bool z, uint32_t tries)
{
	uint32_t i;
	uint8_t tmpreg = 0;
	bool ready;

	for (i = 0; i < tries; ++i)
	{
		_accel_io_read(accel, LSM303C_STATUS_REG_ADDR_A, &tmpreg, 1);
		ready = (x? ((tmpreg & LSM303C_ACC_X_NEWDATA) == LSM303C_ACC_X_NEWDATA): true)
				&& (y? ((tmpreg & LSM303C_ACC_Y_NEWDATA) == LSM303C_ACC_Y_NEWDATA): true)
				&& (z? ((tmpreg & LSM303C_ACC_Z_NEWDATA) == LSM303C_ACC_Z_NEWDATA): true);

		if (ready)
			return true;
	}

	return false;
}

static bool _lsm303c_read(struct accelerometer_driver *__a, float *data)
{
	struct lsm303c_accelerometer *accel = (struct lsm303c_accelerometer *)__a;
	uint8_t tmp_buffer[6] = {0};
	int16_t raw_data[3] = {0};
	uint8_t tmpreg = 0;
	float sensitivity = 0;
	int i = 0;

	_accel_io_read(accel, LSM303C_CTRL_REG1_ADDR_A, &tmpreg, 1);

	if (!_wait_data_ready(accel,
			(tmpreg & LSM303C_ACC_X_ENABLE) == LSM303C_ACC_X_ENABLE,
			(tmpreg & LSM303C_ACC_Y_ENABLE) == LSM303C_ACC_Y_ENABLE,
			(tmpreg & LSM303C_ACC_Z_ENABLE) == LSM303C_ACC_Z_ENABLE,
			0x100))
		return false;

	_accel_io_read(accel, LSM303C_CTRL_REG4_ADDR_A, &tmpreg, 1);

	if (tmpreg & LSM303C_ACC_INC_ADDRESS_ON_RW)
	{
		_accel_io_read(accel, LSM303C_OUT_X_L_ADDR_A, tmp_buffer, 6);
	}
	else
	{
		_accel_io_read(accel, LSM303C_OUT_X_L_ADDR_A, &tmp_buffer[0], 1);
		_accel_io_read(accel, LSM303C_OUT_X_H_ADDR_A, &tmp_buffer[1], 1);
		_accel_io_read(accel, LSM303C_OUT_Y_L_ADDR_A, &tmp_buffer[2], 1);
		_accel_io_read(accel, LSM303C_OUT_Y_H_ADDR_A, &tmp_buffer[3], 1);
		_accel_io_read(accel, LSM303C_OUT_Z_L_ADDR_A, &tmp_buffer[4], 1);
		_accel_io_read(accel, LSM303C_OUT_Z_H_ADDR_A, &tmp_buffer[5], 1);
	}

	for (i = 0; i < 3; i++)
	{
		raw_data[i] = (int16_t)(((uint16_t)tmp_buffer[2*i + 1] << 8) + tmp_buffer[2*i]);
	}

	switch (tmpreg & LSM303C_ACC_FULLSCALE_MASK)
	{
	case LSM303C_ACC_FULLSCALE_2G:
		sensitivity = LSM303C_ACC_SENSITIVITY_2G;
		break;

	case LSM303C_ACC_FULLSCALE_4G:
		sensitivity = LSM303C_ACC_SENSITIVITY_4G;
		break;

	case LSM303C_ACC_FULLSCALE_8G:
		sensitivity = LSM303C_ACC_SENSITIVITY_8G;
		break;
	}

	for (i = 0; i < 3; i++)
	{
		data[i] = (float)(raw_data[i] * sensitivity);
	}

	return true;
}

static struct accelerometer_driver _lsm303c_accelerometer_interface = {
		_lsm303c_open,
		_lsm303c_configure,
		_lsm303c_read,
		_lsm303c_close
};


bool lsm303c_accelerometer_make(struct lsm303c_accelerometer *accel, spi_device_t *dev, struct gpio_pin *cs)
{
	accel->interface = _lsm303c_accelerometer_interface;

	memset(&accel->config, 0, sizeof(accel->config));

	accel->dev = dev;
	accel->cs = cs;

	return true;
}
