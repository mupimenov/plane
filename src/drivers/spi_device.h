#ifndef __SPI_DEVICE_H
#define __SPI_DEVICE_H

#include "block_device.h"

typedef struct block_device spi_device_t;

#define SPI_DEVICE_PARAMETER_CLOCK 0x01

#define SPI_DEVICE_PARAMETER_BITS 0x02

enum spi_config_bits
{
	SPI_8_BIT = 8,
	SPI_9_BIT = 9,
	SPI_10_BIT = 10,
	SPI_11_BIT = 11,
	SPI_12_BIT = 12,
	SPI_13_BIT = 13,
	SPI_14_BIT = 14,
	SPI_15_BIT = 15,
	SPI_16_BIT = 16
};

#define SPI_DEVICE_PARAMETER_PHASE 0x03

enum spi_config_phase
{
	SPI_PHASE_0 = 0,
	SPI_PHASE_1 = 1
};

#define SPI_DEVICE_PARAMETER_POLARITY 0x04

enum spi_config_polarity
{
	SPI_POLARITY_0 = 0,
	SPI_POLARITY_1 = 1
};

#define SPI_DEVICE_PARAMETER_MODE 0x05

enum spi_config_mode
{
	SPI_MODE_SLAVE = 0,
	SPI_MODE_MASTER = 1
};

#define SPI_DEVICE_PARAMETER_DIRECTION 0x06

enum spi_config_direction
{
	SPI_DIRECTION_FULL_DUPLEX = 0,
	SPI_DIRECTION_HALF_DUPLEX_RX = 1,
	SPI_DIRECTION_HALF_DUPLEX_TX = 2
};

#endif
