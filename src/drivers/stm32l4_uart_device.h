#ifndef __STM32L4_UART_DEVICE_H
#define __STM32L4_UART_DEVICE_H

#include "uart_device.h"
#include "fifo.h"

struct stm32l4_uart_device
{
	struct block_device interface;

	struct {
		unsigned long baudrate;
		unsigned char bits;
		unsigned char stop_bit;
		unsigned char parity_enable;
		unsigned char parity_mode;
		unsigned char half_duplex;

		unsigned char dma_enable;
		unsigned long rx_timeout;
	} config;

	unsigned char id;

	struct fifo *rx_fifo;

	unsigned long *reg;
	unsigned long *dma;
	unsigned long tx_channel;
	unsigned long rx_channel;
	unsigned long req_num;

	volatile int rx_done;
};

enum uart_ids
{
	USART1n = 0,
	USART2n,
	USART3n,
	UART4n,
	UART5n,
	
	UART_COUNT
};

bool
stm32l4_uart_make(	struct stm32l4_uart_device *uart,
					unsigned char uart_id,
					struct fifo *rx);

void stm32l4_uart_dma_tx_handler(struct stm32l4_uart_device *uart);
void stm32l4_uart_dma_rx_handler(struct stm32l4_uart_device *uart);

void stm32l4_uart_irq_handler(struct stm32l4_uart_device *uart);

#endif
