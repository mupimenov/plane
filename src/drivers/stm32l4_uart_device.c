#include "stm32l4_uart_device.h"

#include <stddef.h>

#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_dma.h"

#include "fifo.h"

#define TRANSFER_BYTE(uart) \
	do \
	{ \
		if (!FIFO_IS_EMPTY(uart->tx_fifo)) \
		{ \
			uart->reg[UxTHR] = FIFO_FRONT(uart->tx_fifo); \
			FIFO_POP(uart->tx_fifo); \
		} \
		else \
		{ \
			FIFO_FLUSH(uart->tx_fifo); \
		} \
	} while (0);

#define RECEIVE_BYTE(uart, byte) FIFO_PUSH(uart->rx_fifo, byte)

#define APPEND_BYTES(uart, count) FIFO_APPEND(uart->rx_fifo, count)

static const USART_TypeDef * 			uart_regs[UART_COUNT] = {
		USART1,
		USART2,
		USART3,
		UART4,
		UART5
};

static const DMA_TypeDef * 				dma_regs[UART_COUNT] = {
		DMA1,
		DMA1,
		DMA1,
		DMA2,
		DMA2
};

static const uint32_t 					dma_tx_channels[UART_COUNT] = {
		LL_DMA_CHANNEL_4,
		LL_DMA_CHANNEL_7,
		LL_DMA_CHANNEL_2,
		LL_DMA_CHANNEL_3,
		LL_DMA_CHANNEL_1
};

static const uint32_t 					dma_rx_channels[UART_COUNT] = {
		LL_DMA_CHANNEL_5,
		LL_DMA_CHANNEL_6,
		LL_DMA_CHANNEL_3,
		LL_DMA_CHANNEL_5,
		LL_DMA_CHANNEL_2
};

static const uint32_t 					dma_request_nums[UART_COUNT] = {
		LL_DMA_REQUEST_2,
		LL_DMA_REQUEST_2,
		LL_DMA_REQUEST_2,
		LL_DMA_REQUEST_2,
		LL_DMA_REQUEST_2
};

void stm32l4_uart_irq_handler(struct stm32l4_uart_device *uart)
{
	USART_TypeDef *reg = (USART_TypeDef *)uart->reg;
	char byte;

	/* UART rx timeout interrupt occurred ---------------------------------------*/
	if (LL_USART_IsActiveFlag_RTO(reg) && LL_USART_IsEnabledIT_RTO(reg))
	{
		LL_USART_ClearFlag_RTO(reg);

		if (uart->config.dma_enable)
		{
			uint16_t bytes = FIFO_SIZE(uart->rx_fifo)
					- LL_DMA_GetDataLength((DMA_TypeDef*)uart->dma, uart->rx_channel);
			APPEND_BYTES(uart, bytes);

			LL_USART_DisableDMAReq_RX(reg);
			uart->rx_done = true;
		}
	}

	/* UART parity error interrupt occurred -------------------------------------*/
	if (LL_USART_IsActiveFlag_PE(reg) && LL_USART_IsEnabledIT_PE(reg))
	{
		LL_USART_ClearFlag_PE(reg);
	}

	/* UART frame error interrupt occurred --------------------------------------*/
	if (LL_USART_IsActiveFlag_FE(reg) && LL_USART_IsEnabledIT_ERROR(reg))
	{
		LL_USART_ClearFlag_FE(reg);
	}

	/* UART noise error interrupt occurred --------------------------------------*/
	if (LL_USART_IsActiveFlag_NE(reg) && LL_USART_IsEnabledIT_ERROR(reg))
	{
		LL_USART_ClearFlag_NE(reg);
	}

	/* UART Over-Run interrupt occurred -----------------------------------------*/
	if (LL_USART_IsActiveFlag_ORE(reg) && LL_USART_IsEnabledIT_ERROR(reg))
	{
		LL_USART_ClearFlag_ORE(reg);
	}

	/* UART wakeup from Stop mode interrupt occurred -------------------------------------*/
	if (LL_USART_IsActiveFlag_WKUP(reg) && LL_USART_IsEnabledIT_WKUP(reg))
	{
		LL_USART_ClearFlag_WKUP(reg);
		// Wake up
	}

	/* UART in mode Receiver ---------------------------------------------------*/
	if (LL_USART_IsActiveFlag_RXNE(reg) && LL_USART_IsEnabledIT_RXNE(reg))
	{
		if (uart->config.parity_enable == UART_DISABLE_PARITY_BIT)
		{
			byte = (char)(reg->RDR & (char)0x00FF);
		}
		else
		{
			byte = (char)(reg->RDR & (char)0x007F);
		}

		RECEIVE_BYTE(uart, byte);
	}


	/* UART in mode Transmitter ------------------------------------------------*/
	if (LL_USART_IsActiveFlag_TXE(reg) && LL_USART_IsEnabledIT_TXE(reg))
	{
		//
	}

	/* UART in mode Transmitter (transmission end) -----------------------------*/
	if (LL_USART_IsActiveFlag_TC(reg) && LL_USART_IsEnabledIT_TC(reg))
	{
		LL_USART_ClearFlag_TC(reg);
	}
}	

void stm32l4_uart_dma_tx_handler(struct stm32l4_uart_device *uart)
{
	USART_TypeDef *reg = (USART_TypeDef *)uart->reg;

	LL_USART_DisableDMAReq_TX(reg);
}

void stm32l4_uart_dma_rx_handler(struct stm32l4_uart_device *uart)
{
	USART_TypeDef *reg = (USART_TypeDef *)uart->reg;

	LL_USART_DisableDMAReq_RX(reg);
	uart->rx_done = true;
}

static bool _uart_setup(struct block_device *dev, unsigned long parameter, unsigned long value);
static bool _uart_configure(struct block_device *dev);
static int _uart_read(struct block_device *dev, unsigned long address, unsigned char *buffer, unsigned short available_size);
static int _uart_write(struct block_device *dev, unsigned long address, const unsigned char *buffer, unsigned short size);

static void _setup_dma_rx(struct stm32l4_uart_device *uart);

bool
stm32l4_uart_make(	struct stm32l4_uart_device *uart,
					unsigned char uart_id,
					struct fifo *rx)
{	
	if (uart_id >= UART_COUNT)
	{
		return false;
	}
	
	uart->interface.open = NULL;
	uart->interface.close = NULL;
	uart->interface.setup = _uart_setup;
	uart->interface.configure = _uart_configure;
	uart->interface.read = _uart_read;
	uart->interface.write = _uart_write;
	
	uart->rx_fifo = rx;
	
	uart->id = uart_id;
	uart->reg = (unsigned long *)uart_regs[uart_id];
	uart->dma = (unsigned long *)dma_regs[uart_id];
	uart->tx_channel = dma_tx_channels[uart_id];
	uart->rx_channel = dma_rx_channels[uart_id];
	uart->req_num = dma_request_nums[uart_id];
	uart->rx_done = false;
	
	FIFO_FLUSH(uart->rx_fifo);
	
	return true;
}

bool _uart_setup(struct block_device *dev, unsigned long parameter, unsigned long value)
{
	struct stm32l4_uart_device *uart = (struct stm32l4_uart_device *)dev;

	switch (parameter)
	{
	case UART_DEVICE_PARAMETER_BAUDRATE:
		uart->config.baudrate = value;
		break;
	case UART_DEVICE_PARAMETER_BITS:
		uart->config.bits = value;
		break;
	case UART_DEVICE_PARAMETER_STOP_BITS:
		uart->config.stop_bit = value;
		break;
	case UART_DEVICE_PARAMETER_PARITY_ENABLE:
		uart->config.parity_enable = value;
		break;
	case UART_DEVICE_PARAMETER_PARITY_MODE:
		uart->config.parity_mode = value;
		break;
	case UART_DEVICE_PARAMETER_HALF_DUPLEX:
		uart->config.half_duplex = value;
		break;
	case UART_DEVICE_PARAMETER_DMA_ENABLE:
		uart->config.dma_enable = value;
		break;
	case UART_DEVICE_PARAMETER_RX_TIMEOUT:
		uart->config.rx_timeout = value;
		break;
	default:
		return false;
	}

	return true;
}

static bool
_uart_configure(struct block_device *dev)
{	
	struct stm32l4_uart_device *uart = (struct stm32l4_uart_device *)dev;
	USART_TypeDef *reg = (USART_TypeDef *)uart->reg;
	LL_USART_InitTypeDef init_struct;

	init_struct.BaudRate = uart->config.baudrate;

	switch (uart->config.bits)
	{
	case UART_6_BIT:
		if (!uart->config.parity_enable)
			return false;
		init_struct.DataWidth = LL_USART_DATAWIDTH_7B;
		break;
	case UART_7_BIT:
		if (!uart->config.parity_enable)
			init_struct.DataWidth = LL_USART_DATAWIDTH_7B;
		else
			init_struct.DataWidth = LL_USART_DATAWIDTH_8B;
		break;
	case UART_8_BIT:
		if (!uart->config.parity_enable)
			init_struct.DataWidth = LL_USART_DATAWIDTH_8B;
		else
			init_struct.DataWidth = LL_USART_DATAWIDTH_9B;
		break;
	case UART_9_BIT:
		if (uart->config.parity_enable)
			return false;
		init_struct.DataWidth = LL_USART_DATAWIDTH_9B;
		break;
	default:
		return false;
	}

	switch (uart->config.stop_bit)
	{
	case UART_1_STOP_BIT:
		init_struct.StopBits = LL_USART_STOPBITS_1;
		break;
	case UART_2_STOP_BITS:
		init_struct.StopBits = LL_USART_STOPBITS_2;
		break;
	default:
		return false;
	}

	if (uart->config.parity_enable)
	{
		switch (uart->config.parity_mode)
		{
		case UART_PARITY_ODD:
			init_struct.StopBits = LL_USART_PARITY_ODD;
			break;
		case UART_PARITY_EVEN:
			init_struct.StopBits = LL_USART_PARITY_EVEN;
			break;
		default:
			return false;
		}
	}
	else
	{
		init_struct.Parity = LL_USART_PARITY_NONE;
	}

	init_struct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	init_struct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	init_struct.OverSampling = LL_USART_OVERSAMPLING_16;

	LL_USART_Disable(reg);

	if (LL_USART_Init(reg, &init_struct) != SUCCESS)
		return false;

	if (uart->config.rx_timeout)
	{
		LL_USART_SetRxTimeout(reg, uart->config.rx_timeout);
		LL_USART_EnableRxTimeout(reg);
	}

	LL_USART_EnableIT_PE((USART_TypeDef*)uart->reg);
	LL_USART_EnableIT_ERROR((USART_TypeDef*)uart->reg);

	if (uart->config.dma_enable == UART_DMA_ENABLE)
	{
		_setup_dma_rx(uart);

		if (uart->config.rx_timeout)
		{
			LL_USART_EnableIT_RTO((USART_TypeDef*)uart->reg);
		}
		else
		{
			LL_USART_DisableIT_RTO((USART_TypeDef*)uart->reg);
		}
	}
	else
	{
		LL_USART_EnableIT_RXNE((USART_TypeDef*)uart->reg);
	}

	LL_USART_Enable(reg);

	return true;
}

static void _setup_dma_rx(struct stm32l4_uart_device *uart)
{
	USART_TypeDef *reg = (USART_TypeDef *)uart->reg;
	LL_DMA_InitTypeDef init;

	init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	init.Mode = LL_DMA_MODE_NORMAL;

	init.PeriphOrM2MSrcAddress = (uint32_t)&reg->RDR;
	init.PeriphOrM2MSrcDataSize= LL_DMA_PDATAALIGN_HALFWORD;
	init.PeriphOrM2MSrcIncMode= LL_DMA_PERIPH_NOINCREMENT;

	init.MemoryOrM2MDstAddress = (uint32_t)uart->rx_fifo->buf;
	init.MemoryOrM2MDstDataSize= uart->config.bits <= UART_8_BIT? LL_DMA_MDATAALIGN_BYTE: LL_DMA_MDATAALIGN_HALFWORD;
	init.MemoryOrM2MDstIncMode= LL_DMA_MEMORY_INCREMENT;

	init.NbData = uart->rx_fifo->size;

	init.PeriphRequest = uart->req_num;
	init.Priority = LL_DMA_PRIORITY_MEDIUM;

	LL_DMA_DisableChannel((DMA_TypeDef*)uart->dma, uart->rx_channel);

	LL_DMA_Init((DMA_TypeDef*)uart->dma, uart->rx_channel, &init);

	LL_USART_EnableDMAReq_RX(reg);

	LL_DMA_EnableIT_TC((DMA_TypeDef*)uart->dma, uart->rx_channel);
	LL_DMA_EnableChannel((DMA_TypeDef*)uart->dma, uart->rx_channel);
}

static int 
_uart_read(struct block_device *dev, unsigned long address, unsigned char *buffer, unsigned short available_size)
{
	volatile int already_read = 0;
	struct stm32l4_uart_device *uart = (struct stm32l4_uart_device *)dev;
	USART_TypeDef *reg = (USART_TypeDef *)uart->reg;

	if (uart == NULL || reg == NULL)
		return 0;

	if (uart->config.dma_enable)
	{
		if (uart->rx_done)
		{
			while (!FIFO_IS_EMPTY(uart->rx_fifo))
			{
				buffer[already_read++] = FIFO_FRONT(uart->rx_fifo);
				FIFO_POP(uart->rx_fifo);

				if (already_read >= available_size)
					break;
			}

			if (FIFO_IS_EMPTY(uart->rx_fifo))
			{
				uart->rx_done = false;

				FIFO_FLUSH(uart->rx_fifo);

				_setup_dma_rx(uart);
			}
		}
	}
	else
	{
		LL_USART_DisableIT_RXNE(reg);

		while (!FIFO_IS_EMPTY(uart->rx_fifo))
		{
			buffer[already_read++] = FIFO_FRONT(uart->rx_fifo);
			FIFO_POP(uart->rx_fifo);

			if (already_read >= available_size)
				break;
		}

		if (FIFO_IS_EMPTY(uart->rx_fifo))
			FIFO_FLUSH(uart->rx_fifo);

		LL_USART_EnableIT_RXNE(reg);
	}
	
	return already_read;
}

static int 
_uart_write(struct block_device *dev, unsigned long address, const unsigned char *buffer, unsigned short size)
{
	volatile unsigned short already_write = 0;
	struct stm32l4_uart_device *uart = (struct stm32l4_uart_device *)dev;
	USART_TypeDef *reg = (USART_TypeDef *)uart->reg;

	if (uart == NULL || uart->reg == NULL)
		return 0;

	if (uart->config.dma_enable)
	{
		LL_DMA_InitTypeDef init;

		while (LL_USART_IsEnabledDMAReq_TX(reg));

		init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
		init.Mode = LL_DMA_MODE_NORMAL;

		init.MemoryOrM2MDstAddress = (uint32_t)buffer;
		init.MemoryOrM2MDstDataSize = uart->config.bits <= UART_8_BIT? LL_DMA_MDATAALIGN_BYTE: LL_DMA_MDATAALIGN_HALFWORD;
		init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;

		init.PeriphOrM2MSrcAddress = (uint32_t)&reg->TDR;
		init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
		init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;

		init.NbData = size;

		init.PeriphRequest = uart->req_num;
		init.Priority = LL_DMA_PRIORITY_MEDIUM;

		LL_DMA_DisableChannel((DMA_TypeDef*)uart->dma, uart->tx_channel);

		LL_DMA_Init((DMA_TypeDef*)uart->dma, uart->tx_channel, &init);
		LL_DMA_EnableIT_TC((DMA_TypeDef*)uart->dma, uart->tx_channel);

		LL_USART_EnableDMAReq_TX(reg);
		LL_USART_ClearFlag_TC(reg);

		LL_DMA_EnableChannel((DMA_TypeDef*)uart->dma, uart->tx_channel);
	}
	else
	{
		if (uart->config.half_duplex)
			LL_USART_DisableIT_RXNE(reg);

		while (already_write < size)
		{
			if (LL_USART_IsActiveFlag_TXE(reg))
			{
				reg->TDR = (buffer[already_write] & (uint8_t)0xFF);
				already_write++;

				if (uart->config.half_duplex)
				{
					while (!LL_USART_IsActiveFlag_TXE(reg));
					(void)reg->RDR;
				}
			}
		}

		if (uart->config.half_duplex)
			LL_USART_EnableIT_RXNE(reg);
	}

	return already_write;
}
