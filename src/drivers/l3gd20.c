#include "l3gd20.h"

#include <string.h>

#define I_AM_L3GD20   					0xD4
#define I_AM_L3GD20_TR 					0xD5

#define L3GD20_WHO_AM_I_ADDR          	0x0F  /* device identification register */
#define L3GD20_CTRL_REG1_ADDR         	0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         	0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         	0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         	0x23  /* Control register 4 */
#define L3GD20_CTRL_REG5_ADDR         	0x24  /* Control register 5 */
#define L3GD20_REFERENCE_REG_ADDR     	0x25  /* Reference register */
#define L3GD20_OUT_TEMP_ADDR          	0x26  /* Out temp register */
#define L3GD20_STATUS_REG_ADDR        	0x27  /* Status register */
#define L3GD20_OUT_X_L_ADDR           	0x28  /* Output Register X */
#define L3GD20_OUT_X_H_ADDR           	0x29  /* Output Register X */
#define L3GD20_OUT_Y_L_ADDR           	0x2A  /* Output Register Y */
#define L3GD20_OUT_Y_H_ADDR           	0x2B  /* Output Register Y */
#define L3GD20_OUT_Z_L_ADDR           	0x2C  /* Output Register Z */
#define L3GD20_OUT_Z_H_ADDR           	0x2D  /* Output Register Z */
#define L3GD20_FIFO_CTRL_REG_ADDR     	0x2E  /* Fifo control Register */
#define L3GD20_FIFO_SRC_REG_ADDR      	0x2F  /* Fifo src Register */

#define L3GD20_INT1_CFG_ADDR          	0x30  /* Interrupt 1 configuration Register */
#define L3GD20_INT1_SRC_ADDR          	0x31  /* Interrupt 1 source Register */
#define L3GD20_INT1_TSH_XH_ADDR       	0x32  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_XL_ADDR       	0x33  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_YH_ADDR       	0x34  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_YL_ADDR       	0x35  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_ZH_ADDR       	0x36  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_TSH_ZL_ADDR       	0x37  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_DURATION_ADDR     	0x38  /* Interrupt 1 DURATION register */

/* Read/Write command */
#define READWRITE_CMD ((uint8_t)0x80)

/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD ((uint8_t)0x40)

// STATUS_REG
#define L3GD20_ZYX_OVERRUN 				0x80
#define L3GD20_Z_OVERRUN 				0x40
#define L3GD20_Y_OVERRUN 				0x20
#define L3GD20_X_OVERRUN 				0x10
#define L3GD20_ZYX_NEWDATA 				0x08
#define L3GD20_Z_NEWDATA 				0x04
#define L3GD20_Y_NEWDATA 				0x02
#define L3GD20_X_NEWDATA 				0x01

// FIFO_SRC_REG
#define L3GD20_FIFO_GE_WATERMARK 		0x80
#define L3GD20_OVERRUN_STATUS 			0x40
#define L3GD20_FIFO_EMPTY_STATUS 		0x20
#define L3GD20_FIFO_STORED_DATA_LEVEL(x) ((x) & 0x1F)

// INT1_SRC
#define L3GD20_INT1_ACTIVE 				0x40
#define L3GD20_Z_HIGH 					0x20
#define L3GD20_Z_LOW 					0x10
#define L3GD20_Y_HIGH 					0x08
#define L3GD20_Y_LOW 					0x04
#define L3GD20_X_HIGH 					0x02
#define L3GD20_X_LOW 					0x01

#define L3GD20_SENSITIVITY_250DPS  ((float)8.75f)         /*!< gyroscope sensitivity with 250 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_500DPS  ((float)17.50f)        /*!< gyroscope sensitivity with 500 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_2000DPS ((float)70.00f)        /*!< gyroscope sensitivity with 2000 dps full scale [DPS/LSB] */

static bool _gyro_io_init(struct l3gd20_gyro *gyro)
{
	do
	{
		if (!gpio_configure(gyro->cs))
			break;
		gpio_set(gyro->cs);

		if (!device_open(gyro->dev))
			break;

		device_setup(gyro->dev, SPI_DEVICE_PARAMETER_CLOCK, 1000000UL);
		device_setup(gyro->dev, SPI_DEVICE_PARAMETER_BITS, SPI_8_BIT);
		device_setup(gyro->dev, SPI_DEVICE_PARAMETER_PHASE, SPI_PHASE_0);
		device_setup(gyro->dev, SPI_DEVICE_PARAMETER_POLARITY, SPI_POLARITY_0);
		device_setup(gyro->dev, SPI_DEVICE_PARAMETER_MODE, SPI_MODE_MASTER);
		device_setup(gyro->dev, SPI_DEVICE_PARAMETER_DIRECTION, SPI_DIRECTION_FULL_DUPLEX);

		if (!device_configure(gyro->dev))
			break;

		return true;
	} while (0);

	return false;
}

static bool _gyro_io_deinit(struct l3gd20_gyro *gyro)
{
	return device_close(gyro->dev);
}

static void _gyro_io_write(struct l3gd20_gyro *gyro, uint8_t address, const uint8_t *data, uint16_t size)
{
	/* Configure the MS bit:
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
	 */
	if (size > 0x01)
	{
		address |= (uint8_t)MULTIPLEBYTE_CMD;
	}

	/* Set chip select Low at the start of the transmission */
	gpio_clear(gyro->cs);

	/* Send the Address of the indexed register */
	device_write(gyro->dev, 0, &address, 1);

	/* Send the data that will be written into the device (MSB First) */
	device_write(gyro->dev, 0, data, size);

	/* Set chip select High at the end of the transmission */
	gpio_set(gyro->cs);
}

static void _gyro_io_read(struct l3gd20_gyro *gyro, uint8_t address, uint8_t *data, uint16_t size)
{
	if (size > 0x01)
	{
		address |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
	}
	else
	{
		address |= (uint8_t)READWRITE_CMD;
	}

	/* Set chip select Low at the start of the transmission */
	gpio_clear(gyro->cs);

	/* Send the Address of the indexed register */
	device_write(gyro->dev, 0, &address, 1);

	/* Receive the data that will be read from the device (MSB First) */
	device_read(gyro->dev, 0, data, size);

	/* Set chip select High at the end of the transmission */
	gpio_set(gyro->cs);
}

static bool _l3gd20_open(struct gyro_driver *__g)
{  
	struct l3gd20_gyro *gyro = (struct l3gd20_gyro *)__g;

	do
	{
		if (!_gyro_io_init(gyro))
			break;

		return true;
	} while (0);

	return false;
}

static bool _l3gd20_close(struct gyro_driver *__g)
{
	struct l3gd20_gyro *gyro = (struct l3gd20_gyro *)__g;

	do
	{
		if (!_gyro_io_deinit(gyro))
			break;

		return true;
	} while (0);

	return false;
}

static bool _l3gd20_configure(struct gyro_driver *__g)
{
	struct l3gd20_gyro *gyro = (struct l3gd20_gyro *)__g;

	do
	{
		uint8_t tmp;

		_gyro_io_read(gyro, L3GD20_WHO_AM_I_ADDR, &tmp, 1);
		if (tmp != I_AM_L3GD20 && tmp != I_AM_L3GD20_TR)
			break;

		if (gyro->config.reg_selector & L3GD20_SELECT_REG1)
		{
			_gyro_io_write(gyro, L3GD20_CTRL_REG1_ADDR, &gyro->config.reg1, 1);
		}

		if (gyro->config.reg_selector & L3GD20_SELECT_REG2)
		{
			_gyro_io_write(gyro, L3GD20_CTRL_REG2_ADDR, &gyro->config.reg2, 1);
		}

		if (gyro->config.reg_selector & L3GD20_SELECT_REG3)
		{
			_gyro_io_write(gyro, L3GD20_CTRL_REG3_ADDR, &gyro->config.reg3, 1);
		}

		if (gyro->config.reg_selector & L3GD20_SELECT_REG4)
		{
			_gyro_io_write(gyro, L3GD20_CTRL_REG4_ADDR, &gyro->config.reg4, 1);
		}

		if (gyro->config.reg_selector & L3GD20_SELECT_REG5)
		{
			_gyro_io_write(gyro, L3GD20_CTRL_REG5_ADDR, &gyro->config.reg5, 1);
		}

		if (gyro->config.reg_selector & L3GD20_SELECT_FIFO_CTRL_REG) {
			_gyro_io_write(gyro, L3GD20_FIFO_CTRL_REG_ADDR, &gyro->config.fifo_ctrl_reg, 1);
		}

		if (gyro->config.reg_selector & L3GD20_SELECT_INT1_CFG)
		{
			_gyro_io_write(gyro, L3GD20_INT1_CFG_ADDR, &gyro->config.int1_cfg, 1);
		}

		return true;
	} while (0);

	return false;
}

static bool _wait_data_ready(struct l3gd20_gyro *gyro, bool x, bool y, bool z, uint32_t tries)
{
	uint32_t i;
	uint8_t tmpreg = 0;
	bool ready;

	for (i = 0; i < tries; ++i)
	{
		_gyro_io_read(gyro, L3GD20_STATUS_REG_ADDR, &tmpreg, 1);
		ready = (x? ((tmpreg & L3GD20_X_NEWDATA) == L3GD20_X_NEWDATA): true)
				&& (y? ((tmpreg & L3GD20_Y_NEWDATA) == L3GD20_Y_NEWDATA): true)
				&& (z? ((tmpreg & L3GD20_Z_NEWDATA) == L3GD20_Z_NEWDATA): true);

		if (ready)
			return true;
	}

	return false;
}

static bool _l3gd20_read(struct gyro_driver *__g, float *data)
{
	struct l3gd20_gyro *gyro = (struct l3gd20_gyro *)__g;
	uint8_t tmp_buffer[6] = {0};
	int16_t raw_data[3] = {0};
	uint8_t tmpreg = 0;
	float sensitivity = 0;
	int i = 0;

	_gyro_io_read(gyro, L3GD20_CTRL_REG1_ADDR, &tmpreg, 1);

	if (!_wait_data_ready(gyro,
			(tmpreg & L3GD20_X_ENABLE) == L3GD20_X_ENABLE,
			(tmpreg & L3GD20_Y_ENABLE) == L3GD20_Y_ENABLE,
			(tmpreg & L3GD20_Z_ENABLE) == L3GD20_Z_ENABLE,
			0x100))
		return false;

	_gyro_io_read(gyro, L3GD20_CTRL_REG4_ADDR, &tmpreg, 1);

#if 0
	_gyro_io_read(gyro, L3GD20_OUT_X_L_ADDR, &tmp_buffer[0], 1);
	_gyro_io_read(gyro, L3GD20_OUT_X_L_ADDR + 1, &tmp_buffer[1], 1);
	_gyro_io_read(gyro, L3GD20_OUT_X_L_ADDR + 2, &tmp_buffer[2], 1);
	_gyro_io_read(gyro, L3GD20_OUT_X_L_ADDR + 3, &tmp_buffer[3], 1);
	_gyro_io_read(gyro, L3GD20_OUT_X_L_ADDR + 4, &tmp_buffer[4], 1);
	_gyro_io_read(gyro, L3GD20_OUT_X_L_ADDR + 5, &tmp_buffer[5], 1);
#else
	_gyro_io_read(gyro, L3GD20_OUT_X_L_ADDR, tmp_buffer, 6);
#endif

	/* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
	if (!(tmpreg & L3GD20_ENDIAN_MSB))
	{
		for (i = 0; i < 3; i++)
		{
			raw_data[i] = (int16_t)(((uint16_t)tmp_buffer[2*i + 1] << 8) + tmp_buffer[2*i]);
		}
	}
	else
	{
		for (i = 0; i < 3; i++)
		{
			raw_data[i] = (int16_t)(((uint16_t)tmp_buffer[2*i] << 8) + tmp_buffer[2*i + 1]);
		}
	}

	/* Switch the sensitivity value set in the CRTL4 */
	switch (tmpreg & L3GD20_FULLSCALE_SELECTION)
	{
	case L3GD20_FULLSCALE_250:
		sensitivity = L3GD20_SENSITIVITY_250DPS * 0.001f;
		break;

	case L3GD20_FULLSCALE_500:
		sensitivity = L3GD20_SENSITIVITY_500DPS * 0.001f;
		break;

	case L3GD20_FULLSCALE_2000:
		sensitivity = L3GD20_SENSITIVITY_2000DPS * 0.001f;
		break;
	}

	for (i = 0; i < 3; i++)
	{
		data[i] = (float)(raw_data[i] * sensitivity);
	}

	return true;
}

static struct gyro_driver _l3gd20_gyro_interface = {
		_l3gd20_open,
		_l3gd20_configure,
		_l3gd20_read,
		_l3gd20_close
};

bool l3gd20_make(struct l3gd20_gyro *gyro, spi_device_t *dev, struct gpio_pin *cs)
{
	gyro->interface = _l3gd20_gyro_interface;

	memset(&gyro->config, 0, sizeof(gyro->config));

	gyro->dev = dev;
	gyro->cs = cs;

	return true;
}
