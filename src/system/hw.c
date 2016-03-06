/*
 * hw.c
 *
 *  Created on: Feb 22, 2016
 *      Author: mupimenov
 */

#include "stm32l4xx_hal.h"

#include "stm32l4_gpio_pin.h"
#include "stm32l4_uart_device.h"
#include "stm32l4_spi_device.h"

#include "stm32l4xx_ll_dma.h"

#include "l3gd20.h"
#include "lsm303c.h"

static struct stm32l4_gpio_pin led_red = GPIO_INSTANCE(GPIOB, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
static struct stm32l4_gpio_pin led_green = GPIO_INSTANCE(GPIOE, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

static struct stm32l4_gpio_pin uart_pins = GPIO_INSTANCE(GPIOD, GPIO_PIN_5 | GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, GPIO_AF7_USART2);

static struct stm32l4_gpio_pin spi_pins = GPIO_INSTANCE(GPIOD, GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI2);
static struct stm32l4_gpio_pin l3d20_cs = GPIO_INSTANCE(GPIOD, GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0);
static struct stm32l4_gpio_pin lsm303c_accel_cs = GPIO_INSTANCE(GPIOE, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0);

static struct l3gd20_gyro gyro;
static struct lsm303c_accelerometer accel;

#define LINK_BUFFER_SIZE 128

FIFO_STATIC(link_fifo, link_buffer, LINK_BUFFER_SIZE);
static struct stm32l4_uart_device link_uart;

static struct stm32l4_spi_device mems_spi;

void hw_led_init(void)
{
	// red
	__GPIOB_CLK_ENABLE();

	gpio_configure(&led_red.interface);

	// green
	__GPIOE_CLK_ENABLE();

	gpio_configure(&led_green.interface);
}

void hw_led_set_red(int state)
{
	if (state)
		gpio_set(&led_red.interface);
	else
		gpio_clear(&led_red.interface);
}

void hw_led_set_green(int state)
{
	if (state)
		gpio_set(&led_green.interface);
	else
		gpio_clear(&led_green.interface);
}

void hw_link_init(void)
{
	// pins
	__GPIOD_CLK_ENABLE();

	gpio_configure(&uart_pins.interface);

	// DMA
	__DMA1_CLK_ENABLE();

	// uart
	__USART2_CLK_ENABLE();

	if (!stm32l4_uart_make(&link_uart, USART2n, &link_fifo))
	{
		hw_led_set_red(1);
		while (1);
	}

	NVIC_EnableIRQ(USART2_IRQn);

	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

uart_device_t *
hw_get_link(void)
{
	return &link_uart.interface;
}

void USART2_IRQHandler(void)
{
	stm32l4_uart_irq_handler(&link_uart);
}

// TX
void DMA1_Channel7_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_TC7(DMA1))
	{
		stm32l4_uart_dma_tx_handler(&link_uart);

		LL_DMA_ClearFlag_TC7(DMA1);
	}
}

// RX
void DMA1_Channel6_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_TC6(DMA1))
	{
		stm32l4_uart_dma_rx_handler(&link_uart);

		LL_DMA_ClearFlag_TC6(DMA1);
	}
}

void hw_mems_init(void)
{
	// pins
	__GPIOD_CLK_ENABLE();
	__GPIOE_CLK_ENABLE();

	gpio_configure(&spi_pins.interface);

	// spi
	__SPI2_CLK_ENABLE();

	do
	{
		if (!stm32l4_spi_make(&mems_spi, SPI2n))
			break;

		// ACCEL

		if (!lsm303c_accelerometer_make(&accel, &mems_spi.interface, &lsm303c_accel_cs.interface))
			break;

		if (!accelerometer_open(&accel.interface))
			break;

		accel.config.reg_selector = LSM303C_SELECT_REG1;
		accel.config.reg1 = /*LSM303C_ACC_HIGH_RESOLUTION_ENABLE
				|*/ LSM303C_ACC_DATA_RATE_400HZ
				| LSM303C_ACC_CONTINUOUS_UPDATE
				| LSM303C_ACC_X_ENABLE
				| LSM303C_ACC_Y_ENABLE
				| LSM303C_ACC_Z_ENABLE;

		accel.config.reg_selector |= LSM303C_SELECT_REG2;
		accel.config.reg2 = LSM303C_ACC_CUTOFF_DATA_RATE_DIV400
				| LSM303C_ACC_HIGHPASS_MODE_NORMAL
				/* | LSM303C_ACC_HIGHPASS_TO_FIFO_ENABLE */;

		accel.config.reg_selector |= LSM303C_SELECT_REG3;
		accel.config.reg3 = LSM303C_ACC_FIFO_ENABLE;

		accel.config.reg_selector |= LSM303C_SELECT_REG4;
		accel.config.reg4 = LSM303C_ACC_AA_FILTER_BANDWIDTH_400HZ
				| LSM303C_ACC_FULLSCALE_4G
				| LSM303C_ACC_SPI_READ_WRITE_ENABLE;

		accel.config.reg_selector |= LSM303C_SELECT_REG5;
		accel.config.reg5 = LSM303C_ACC_NO_DECIMATION;

		//accel.config.reg_selector |= LSM303C_SELECT_REG6;

		accel.config.reg_selector |= LSM303C_SELECT_FIFO_CTRL_REG;
		accel.config.fifo_ctrl_reg = LSM303C_ACC_FIFO_BYPASS_MODE;

		if (!accelerometer_configure(&accel.interface))
			break;

		(void)accelerometer_close(&accel.interface);

		// GYRO

		if (!l3gd20_make(&gyro, &mems_spi.interface, &l3d20_cs.interface))
			break;

		if (!gyro_open(&gyro.interface))
			break;

		gyro.config.reg_selector = L3GD20_SELECT_REG1;
		gyro.config.reg1 = L3GD20_OUTPUT_DATARATE_380HZ
				| L3GD20_BANDWIDTH_50_FOR_DATARATE_380HZ
				| L3GD20_MODE_ACTIVE
				| L3GD20_X_ENABLE
				| L3GD20_Y_ENABLE
				| L3GD20_Z_ENABLE;

		gyro.config.reg_selector |= L3GD20_SELECT_REG2;
		gyro.config.reg2 = L3GD20_HPM_NORMAL_MODE
				| L3GD20_HPFCF_27HZ_FOR_DATARATE_380HZ;

		gyro.config.reg_selector |= L3GD20_SELECT_REG3;
		gyro.config.reg3 = 0x00;

		gyro.config.reg_selector |= L3GD20_SELECT_REG4;
		gyro.config.reg4 = L3GD20_DATA_UPDATE_ONREAD
				| L3GD20_FULLSCALE_500;

		gyro.config.reg_selector |= L3GD20_SELECT_REG5;
		gyro.config.reg5 = L3GD20_FIFO_ENABLE
				| L3GD20_HIGHPASSFILTER_ENABLE;

		gyro.config.reg_selector |= L3GD20_SELECT_FIFO_CTRL_REG;
		gyro.config.fifo_ctrl_reg = L3GD20_FIFO_BYPASS_MODE;

		gyro.config.reg_selector |= L3GD20_SELECT_INT1_CFG;
		gyro.config.int1_cfg = 0x00;

		if (!gyro_configure(&gyro.interface))
			break;

		(void)gyro_close(&gyro.interface);

		// COMPASS

		return;
	} while (0);

	hw_led_set_red(1);
	while (1);
}

struct gyro_driver *
hw_get_gyro(void)
{
	return (struct gyro_driver *)&gyro;
}

struct accelerometer_driver *
hw_get_accel(void)
{
	return (struct accelerometer_driver *)&accel;
}
