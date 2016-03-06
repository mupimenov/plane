/*
 * stm32l4_spi_device.h
 *
 *  Created on: Feb 23, 2016
 *      Author: mupimenov
 */

#ifndef SRC_DRIVERS_STM32L4_SPI_DEVICE_H_
#define SRC_DRIVERS_STM32L4_SPI_DEVICE_H_

#include "spi_device.h"

struct stm32l4_spi_device
{
	struct block_device interface;

	struct
	{
		unsigned long clock;
		unsigned char bits;
		unsigned char phase;
		unsigned char polarity;
		unsigned char mode;
		unsigned char direction;

	} config;

	unsigned char id;

	volatile unsigned long *reg;
};

enum spi_ids
{
	SPI1n = 0,
	SPI2n,
	SPI3n,

	SPI_COUNT
};

bool
stm32l4_spi_make(	struct stm32l4_spi_device *spi,
					unsigned char spi_id);

#endif /* SRC_DRIVERS_STM32L4_SPI_DEVICE_H_ */
