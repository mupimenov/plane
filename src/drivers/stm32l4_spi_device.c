/*
 * stm32l4_spi_device.c
 *
 *  Created on: Feb 23, 2016
 *      Author: mupimenov
 */

#include "stm32l4_spi_device.h"

#include <stddef.h>

#include "stm32l4xx_ll_spi.h"

static const SPI_TypeDef * 			spi_regs[SPI_COUNT] = {
	SPI1,
	SPI2,
	SPI3
};

static bool _spi_setup(struct char_device *dev, unsigned long parameter, unsigned long value);
static bool _spi_configure(struct char_device *dev);
static int _spi_read(struct char_device *dev, unsigned char *buffer, unsigned short available_size);
static int _spi_write(struct char_device *dev, const unsigned char *buffer, unsigned short size);

bool
stm32l4_spi_make(	struct stm32l4_spi_device *spi,
					unsigned char spi_id)
{
	if (spi_id >= SPI_COUNT)
	{
		return false;
	}

	spi->interface.open = NULL;
	spi->interface.close = NULL;
	spi->interface.setup = _spi_setup;
	spi->interface.configure = _spi_configure;
	spi->interface.read = _spi_read;
	spi->interface.write = _spi_write;

	spi->id = spi_id;
	spi->reg = (volatile unsigned long *)spi_regs[spi_id];

	return true;
}

static bool
_spi_setup(struct char_device *dev, unsigned long parameter, unsigned long value)
{
	struct stm32l4_spi_device *spi = (struct stm32l4_spi_device *)dev;

	switch (parameter)
	{
	case SPI_DEVICE_PARAMETER_CLOCK:
		spi->config.clock = value;
		break;
	case SPI_DEVICE_PARAMETER_BITS:
		spi->config.bits = value;
		break;
	case SPI_DEVICE_PARAMETER_PHASE:
		spi->config.phase = value;
		break;
	case SPI_DEVICE_PARAMETER_POLARITY:
		spi->config.polarity = value;
		break;
	case SPI_DEVICE_PARAMETER_MODE:
		spi->config.mode = value;
		break;
	case SPI_DEVICE_PARAMETER_DIRECTION:
		spi->config.direction = value;
		break;
	default:
		return false;
	}

	return true;
}

static bool
_spi_configure(struct char_device *dev)
{
	struct stm32l4_spi_device *spi = (struct stm32l4_spi_device *)dev;
	SPI_TypeDef *reg = (SPI_TypeDef *)spi->reg;
	LL_SPI_InitTypeDef init_struct;

	/**
	 * TODO: generate baud rate prescaler dynamically
	 */
	init_struct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
	switch (spi->config.direction)
	{
	case SPI_DIRECTION_2LINES:
		init_struct.TransferDirection = LL_SPI_FULL_DUPLEX;
		break;
	case SPI_DIRECTION_1LINE:
		init_struct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
		break;
	default:
		return false;
	}

	init_struct.ClockPhase = spi->config.phase == SPI_PHASE_0? LL_SPI_PHASE_1EDGE: LL_SPI_PHASE_2EDGE;
	init_struct.ClockPolarity = spi->config.polarity == SPI_POLARITY_0? LL_SPI_POLARITY_LOW: LL_SPI_POLARITY_HIGH;
	init_struct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	init_struct.CRCPoly = 7;

	switch (spi->config.bits)
	{
	case SPI_8_BIT:
		init_struct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
		break;
	case SPI_9_BIT:
		init_struct.DataWidth = LL_SPI_DATAWIDTH_9BIT;
		break;
	case SPI_10_BIT:
		init_struct.DataWidth = LL_SPI_DATAWIDTH_10BIT;
		break;
	case SPI_11_BIT:
		init_struct.DataWidth = LL_SPI_DATAWIDTH_11BIT;
		break;
	case SPI_12_BIT:
		init_struct.DataWidth = LL_SPI_DATAWIDTH_12BIT;
		break;
	case SPI_13_BIT:
		init_struct.DataWidth = LL_SPI_DATAWIDTH_13BIT;
		break;
	case SPI_14_BIT:
		init_struct.DataWidth = LL_SPI_DATAWIDTH_14BIT;
		break;
	case SPI_15_BIT:
		init_struct.DataWidth = LL_SPI_DATAWIDTH_15BIT;
		break;
	case SPI_16_BIT:
		init_struct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
		break;
	default:
		return false;
	}

	init_struct.BitOrder = LL_SPI_MSB_FIRST;
	init_struct.NSS = LL_SPI_NSS_SOFT;
	init_struct.Mode = LL_SPI_MODE_MASTER;

	LL_SPI_Disable(reg);

	if (LL_SPI_Init(reg, &init_struct) != SUCCESS)
		return false;

	LL_SPI_SetStandard(reg, LL_SPI_PROTOCOL_MOTOROLA);

	return true;
}

static void
_end_rx_transaction(SPI_TypeDef *reg)
{
	while (LL_SPI_IsActiveFlag_BSY(reg))
	{

	}

	while (LL_SPI_GetRxFIFOLevel(reg) != LL_SPI_RX_FIFO_EMPTY)
	{
		__IO uint8_t tmpreg = LL_SPI_ReceiveData8(reg);
		((void)(tmpreg));
	}

	LL_SPI_ClearFlag_OVR(reg);
}

static int _full_duplex_read(struct char_device *dev, unsigned char *buffer, unsigned short available_size)
{
	struct stm32l4_spi_device *spi = (struct stm32l4_spi_device *)dev;
	SPI_TypeDef *reg = (SPI_TypeDef *)spi->reg;
	volatile unsigned short write_count = 0;
	volatile unsigned short read_count = 0;

	if (spi->config.bits >= SPI_8_BIT && available_size > 1)
	{
		LL_SPI_SetRxFIFOThreshold(reg, LL_SPI_RX_FIFO_TH_HALF);
	}
	else
	{
		LL_SPI_SetRxFIFOThreshold(reg, LL_SPI_RX_FIFO_TH_QUARTER);
	}

	if (!LL_SPI_IsEnabled(reg))
	{
		LL_SPI_Enable(reg);
	}

	while (read_count < available_size)
	{
		if (LL_SPI_IsActiveFlag_TXE(reg))
		{
			if (spi->config.bits >= SPI_8_BIT && ((available_size - write_count) > 1))
			{
				reg->DR = (uint16_t)buffer[write_count] | ((uint16_t)buffer[write_count + 1] << 8);
				write_count += 2;
			}
			else if (spi->config.bits > SPI_8_BIT && ((available_size - write_count) == 1))
			{
				reg->DR = (uint16_t)buffer[write_count];
				++write_count;
			}
			else if (spi->config.bits == SPI_8_BIT && ((available_size - write_count) == 1))
			{
				*(__IO uint8_t *)&reg->DR = buffer[write_count];
				++write_count;
			}
		}

		if (LL_SPI_IsActiveFlag_RXNE(reg))
		{
			if (spi->config.bits >= SPI_8_BIT && ((available_size - read_count) > 1))
			{
				uint16_t data = reg->DR;
				buffer[read_count] = data;
				buffer[read_count + 1] = data >> 8;

				read_count += 2;

				if (spi->config.bits == SPI_8_BIT && ((available_size - read_count) == 1))
				{
					LL_SPI_SetRxFIFOThreshold(reg, LL_SPI_RX_FIFO_TH_QUARTER);
				}
			}
			else if (spi->config.bits > SPI_8_BIT && ((available_size - read_count) == 1))
			{
				uint16_t data = reg->DR & 0xFF;
				buffer[read_count] = data;
				buffer[read_count + 1] = data >> 8;

				++read_count;
			}
			else if (spi->config.bits == SPI_8_BIT && ((available_size - read_count) == 1))
			{
				buffer[read_count] = *(__IO uint8_t *)&reg->DR;

				++read_count;
			}
		}
	}

	LL_SPI_Disable(reg);

	if (spi->config.direction == SPI_DIRECTION_1LINE)
	{
		_end_rx_transaction(reg);
	}

	return read_count;
}

static int
_spi_read(struct char_device *dev, unsigned char *buffer, unsigned short available_size)
{
	struct stm32l4_spi_device *spi = (struct stm32l4_spi_device *)dev;
	SPI_TypeDef *reg = (SPI_TypeDef *)spi->reg;
	volatile unsigned short read_count = 0;

	if (spi == NULL || reg == NULL)
		return 0;

	if (spi->config.direction == SPI_DIRECTION_2LINES)
	{
		return _full_duplex_read(dev, buffer, available_size);
	}
	else
	{
		LL_SPI_SetTransferDirection(reg, LL_SPI_HALF_DUPLEX_RX);
	}

	if (spi->config.bits > SPI_8_BIT)
	{
		LL_SPI_SetRxFIFOThreshold(reg, LL_SPI_RX_FIFO_TH_HALF);
	}
	else
	{
		LL_SPI_SetRxFIFOThreshold(reg, LL_SPI_RX_FIFO_TH_QUARTER);
	}

	if (!LL_SPI_IsEnabled(reg))
	{
		LL_SPI_Enable(reg);
	}

	while (read_count < available_size)
	{
		if (LL_SPI_IsActiveFlag_RXNE(reg))
		{
			if (spi->config.bits > SPI_8_BIT && ((available_size - read_count) > 0))
			{
				uint16_t data = reg->DR;
				buffer[read_count] = data;
				buffer[read_count + 1] = data >> 8;

				read_count += 2;
			}
			else if (spi->config.bits == SPI_8_BIT && ((available_size - read_count) > 0))
			{
				buffer[read_count] = *(__IO uint8_t *)&reg->DR;

				++read_count;
			}
		}
	}

	LL_SPI_Disable(reg);

	if (spi->config.direction == SPI_DIRECTION_1LINE)
	{
		_end_rx_transaction(reg);
	}

	return read_count;
}

static void
_end_tx_transaction(SPI_TypeDef *reg)
{
	while (LL_SPI_GetTxFIFOLevel(reg) != LL_SPI_TX_FIFO_EMPTY)
	{

	}

	while (LL_SPI_IsActiveFlag_BSY(reg))
	{

	}
}

static int
_spi_write(struct char_device *dev, const unsigned char *buffer, unsigned short size)
{
	struct stm32l4_spi_device *spi = (struct stm32l4_spi_device *)dev;
	SPI_TypeDef *reg = (SPI_TypeDef *)spi->reg;
	volatile unsigned short write_count = 0;

	if (spi == NULL || reg == NULL)
		return 0;

	if (spi->config.direction == SPI_DIRECTION_1LINE)
	{
		LL_SPI_SetTransferDirection(reg, LL_SPI_HALF_DUPLEX_TX);
	}

#if 0
	if (spi->config.bits >= SPI_8_BIT && size > 1)
	{
		LL_SPI_SetRxFIFOThreshold(reg, LL_SPI_RX_FIFO_TH_HALF);
	}
	else
	{
		LL_SPI_SetRxFIFOThreshold(reg, LL_SPI_RX_FIFO_TH_QUARTER);
	}
#endif

	if (!LL_SPI_IsEnabled(reg))
	{
		LL_SPI_Enable(reg);
	}

	while (write_count < size)
	{
		if (LL_SPI_IsActiveFlag_TXE(reg))
		{
			if (spi->config.bits >= SPI_8_BIT && ((size - write_count) > 1))
			{
				reg->DR = (uint16_t)buffer[write_count] |
						((uint16_t)buffer[write_count + 1] << 8);
				write_count += 2;
			}
			else if (spi->config.bits > SPI_8_BIT && ((size - write_count) == 1))
			{
				reg->DR = (uint16_t)buffer[write_count];
				++write_count;
			}
			else if (spi->config.bits == SPI_8_BIT && ((size - write_count) == 1))
			{
				*(__IO uint8_t *)&reg->DR = buffer[write_count];
				++write_count;
			}
		}
	}

	_end_tx_transaction(reg);

	LL_SPI_Disable(reg);

	if (spi->config.direction == SPI_DIRECTION_2LINES)
	{
		_end_rx_transaction(reg);
	}

	return write_count;
}
