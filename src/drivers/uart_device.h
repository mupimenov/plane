#ifndef __UART_DEVICE_H
#define __UART_DEVICE_H

#include "block_device.h"

typedef struct block_device uart_device_t;

#define UART_DEVICE_PARAMETER_BAUDRATE 0x01

#define UART_DEVICE_PARAMETER_BITS 0x02

enum uart_config_bits
{
	UART_5_BIT = 0,
	UART_6_BIT = 1,
	UART_7_BIT = 2,
	UART_8_BIT = 3,
	UART_9_BIT = 4
};

#define UART_DEVICE_PARAMETER_STOP_BITS 0x03

enum uart_config_stop_bits
{
	UART_1_STOP_BIT = 0,
	UART_2_STOP_BITS = 1
};

#define UART_DEVICE_PARAMETER_PARITY_ENABLE 0x04

enum uart_config_parity_enable
{
	UART_DISABLE_PARITY_BIT = 0,
	UART_ENABLE_PARITY_BIT = 1
};

#define UART_DEVICE_PARAMETER_PARITY_MODE 0x05

enum uart_config_parity_mode
{
	UART_PARITY_ODD = 0,
	UART_PARITY_EVEN = 1,
	UART_PARITY_1 = 2,
	UART_PARITY_0 = 3
};

#define UART_DEVICE_PARAMETER_HALF_DUPLEX 0x06

enum uart_config_half_duplex
{
	UART_FULL_DUPLEX = 0,
	UART_HALF_DUPLEX = 1
};


#define UART_DEVICE_PARAMETER_DMA_ENABLE 0x07

enum uart_config_dma_enable {
	UART_DMA_DISABLE = 0,
	UART_DMA_ENABLE = 1
};

#define UART_DEVICE_PARAMETER_RX_TIMEOUT 0x08

#endif
