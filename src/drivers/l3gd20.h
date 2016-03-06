#ifndef __L3GD20_H
#define __L3GD20_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "gyro.h"
#include "gpio.h"
#include "spi_device.h"

struct l3gd20_gyro {
	struct gyro_driver interface;

	struct {
		uint32_t reg_selector;

		uint8_t reg1;
		uint8_t reg2;
		uint8_t reg3;
		uint8_t reg4;
		uint8_t reg5;
		uint8_t fifo_ctrl_reg;
		uint8_t int1_cfg;
	} config;

	spi_device_t *dev;
	struct gpio_pin *cs;
};

// REG_SELECTOR
#define L3GD20_SELECT_REG1								((uint32_t)0x00000001)
#define L3GD20_SELECT_REG2								((uint32_t)0x00000002)
#define L3GD20_SELECT_REG3								((uint32_t)0x00000004)
#define L3GD20_SELECT_REG4								((uint32_t)0x00000008)
#define L3GD20_SELECT_REG5								((uint32_t)0x00000010)
#define L3GD20_SELECT_FIFO_CTRL_REG						((uint32_t)0x00000020)
#define L3GD20_SELECT_INT1_CFG							((uint32_t)0x00000040)

// REG1
#define L3GD20_OUTPUT_DATARATE_95HZ    					((uint8_t)0x00)
#define L3GD20_OUTPUT_DATARATE_190HZ    				((uint8_t)0x40)
#define L3GD20_OUTPUT_DATARATE_380HZ    				((uint8_t)0x80)
#define L3GD20_OUTPUT_DATARATE_760HZ    				((uint8_t)0xC0)
#define L3GD20_BANDWIDTH_12_5_FOR_DATARATE_95HZ 		((uint8_t)0x00)
#define L3GD20_BANDWIDTH_25_FOR_DATARATE_95HZ   		((uint8_t)0x10)
#define L3GD20_BANDWIDTH_12_5_FOR_DATARATE_190HZ 		((uint8_t)0x00)
#define L3GD20_BANDWIDTH_25_FOR_DATARATE_190HZ   		((uint8_t)0x10)
#define L3GD20_BANDWIDTH_50_FOR_DATARATE_190HZ     		((uint8_t)0x20)
#define L3GD20_BANDWIDTH_70_FOR_DATARATE_190HZ      	((uint8_t)0x30)
#define L3GD20_BANDWIDTH_20_FOR_DATARATE_380HZ 			((uint8_t)0x00)
#define L3GD20_BANDWIDTH_25_FOR_DATARATE_380HZ   		((uint8_t)0x10)
#define L3GD20_BANDWIDTH_50_FOR_DATARATE_380HZ     		((uint8_t)0x20)
#define L3GD20_BANDWIDTH_100_FOR_DATARATE_380HZ     	((uint8_t)0x30)
#define L3GD20_BANDWIDTH_30_FOR_DATARATE_760HZ 			((uint8_t)0x00)
#define L3GD20_BANDWIDTH_35_FOR_DATARATE_760HZ   		((uint8_t)0x10)
#define L3GD20_BANDWIDTH_50_FOR_DATARATE_760HZ     		((uint8_t)0x20)
#define L3GD20_BANDWIDTH_100_FOR_DATARATE_760HZ     	((uint8_t)0x30)
#define L3GD20_MODE_POWERDOWN       					((uint8_t)0x00)
#define L3GD20_MODE_ACTIVE          					((uint8_t)0x08)
#define L3GD20_X_ENABLE            						((uint8_t)0x02)
#define L3GD20_Y_ENABLE            						((uint8_t)0x01)
#define L3GD20_Z_ENABLE            						((uint8_t)0x04)
// or
#define L3GD20_AXES_ENABLE         						((uint8_t)0x07)
#define L3GD20_AXES_DISABLE        						((uint8_t)0x00)

// REG2
#define L3GD20_HPM_NORMAL_MODE_RES         				((uint8_t)0x00)
#define L3GD20_HPM_REF_SIGNAL              				((uint8_t)0x10)
#define L3GD20_HPM_NORMAL_MODE             				((uint8_t)0x20)
#define L3GD20_HPM_AUTORESET_INT           				((uint8_t)0x30)

#define L3GD20_HPFCF_7_2HZ_FOR_DATARATE_95HZ            ((uint8_t)0x00)
#define L3GD20_HPFCF_3_5HZ_FOR_DATARATE_95HZ            ((uint8_t)0x01)
#define L3GD20_HPFCF_1_8HZ_FOR_DATARATE_95HZ            ((uint8_t)0x02)
#define L3GD20_HPFCF_0_9HZ_FOR_DATARATE_95HZ            ((uint8_t)0x03)
#define L3GD20_HPFCF_0_45HZ_FOR_DATARATE_95HZ          	((uint8_t)0x04)
#define L3GD20_HPFCF_0_18HZ_FOR_DATARATE_95HZ           ((uint8_t)0x05)
#define L3GD20_HPFCF_0_09HZ_FOR_DATARATE_95HZ           ((uint8_t)0x06)
#define L3GD20_HPFCF_0_045HZ_FOR_DATARATE_95HZ          ((uint8_t)0x07)
#define L3GD20_HPFCF_0_018HZ_FOR_DATARATE_95HZ          ((uint8_t)0x08)
#define L3GD20_HPFCF_0_009HZ_FOR_DATARATE_95HZ          ((uint8_t)0x09)

#define L3GD20_HPFCF_13_7HZ_FOR_DATARATE_190HZ          ((uint8_t)0x00)
#define L3GD20_HPFCF_7_2HZ_FOR_DATARATE_190HZ           ((uint8_t)0x01)
#define L3GD20_HPFCF_3_5HZ_FOR_DATARATE_190HZ           ((uint8_t)0x02)
#define L3GD20_HPFCF_1_8HZ_FOR_DATARATE_190HZ           ((uint8_t)0x03)
#define L3GD20_HPFCF_0_9HZ_FOR_DATARATE_190HZ          	((uint8_t)0x04)
#define L3GD20_HPFCF_0_45HZ_FOR_DATARATE_190HZ          ((uint8_t)0x05)
#define L3GD20_HPFCF_0_18HZ_FOR_DATARATE_190HZ          ((uint8_t)0x06)
#define L3GD20_HPFCF_0_09HZ_FOR_DATARATE_190HZ          ((uint8_t)0x07)
#define L3GD20_HPFCF_0_045HZ_FOR_DATARATE_190HZ         ((uint8_t)0x08)
#define L3GD20_HPFCF_0_018HZ_FOR_DATARATE_190HZ         ((uint8_t)0x09)

#define L3GD20_HPFCF_27HZ_FOR_DATARATE_380HZ          	((uint8_t)0x00)
#define L3GD20_HPFCF_13_7HZ_FOR_DATARATE_380HZ          ((uint8_t)0x01)
#define L3GD20_HPFCF_7_2HZ_FOR_DATARATE_380HZ           ((uint8_t)0x02)
#define L3GD20_HPFCF_3_5HZ_FOR_DATARATE_380HZ           ((uint8_t)0x03)
#define L3GD20_HPFCF_1_8HZ_FOR_DATARATE_380HZ           ((uint8_t)0x04)
#define L3GD20_HPFCF_0_9HZ_FOR_DATARATE_380HZ          	((uint8_t)0x05)
#define L3GD20_HPFCF_0_45HZ_FOR_DATARATE_380HZ          ((uint8_t)0x06)
#define L3GD20_HPFCF_0_18HZ_FOR_DATARATE_380HZ          ((uint8_t)0x07)
#define L3GD20_HPFCF_0_09HZ_FOR_DATARATE_380HZ          ((uint8_t)0x08)
#define L3GD20_HPFCF_0_045HZ_FOR_DATARATE_380HZ         ((uint8_t)0x09)

#define L3GD20_HPFCF_51_4HZ_FOR_DATARATE_760HZ          ((uint8_t)0x00)
#define L3GD20_HPFCF_27HZ_FOR_DATARATE_760HZ          	((uint8_t)0x01)
#define L3GD20_HPFCF_13_7HZ_FOR_DATARATE_760HZ          ((uint8_t)0x02)
#define L3GD20_HPFCF_7_2HZ_FOR_DATARATE_760HZ           ((uint8_t)0x03)
#define L3GD20_HPFCF_3_5HZ_FOR_DATARATE_760HZ           ((uint8_t)0x04)
#define L3GD20_HPFCF_1_8HZ_FOR_DATARATE_760HZ           ((uint8_t)0x05)
#define L3GD20_HPFCF_0_9HZ_FOR_DATARATE_760HZ          	((uint8_t)0x06)
#define L3GD20_HPFCF_0_45HZ_FOR_DATARATE_760HZ          ((uint8_t)0x07)
#define L3GD20_HPFCF_0_18HZ_FOR_DATARATE_760HZ          ((uint8_t)0x08)
#define L3GD20_HPFCF_0_09HZ_FOR_DATARATE_760HZ          ((uint8_t)0x09)

// REG3
#define L3GD20_INT1INTERRUPT_ENABLE        				((uint8_t)0x80) // if 0x00 then disable
#define L3GD20_INT1INTERRUPT_BOOT        				((uint8_t)0x40)
#define L3GD20_INT1INTERRUPT_ACTIVE        				((uint8_t)0x20)
#define L3GD20_INTERRUPT_OPEN_DRAIN        				((uint8_t)0x10) // if 0x00 then push pull
#define L3GD20_INT2INTERRUPT_DATAREADY     				((uint8_t)0x08)
#define L3GD20_INT2INTERRUPT_WATERMARK     				((uint8_t)0x04)
#define L3GD20_INT2INTERRUPT_OVERRUN     				((uint8_t)0x02)
#define L3GD20_INT2INTERRUPT_EMPTY     					((uint8_t)0x01)

// REG4
#define L3GD20_DATA_UPDATE_ONREAD      					((uint8_t)0x80) // if 0x00 then continuous
#define L3GD20_ENDIAN_MSB	                			((uint8_t)0x40) // if 0x00 then LSB
#define L3GD20_FULLSCALE_250       						((uint8_t)0x00)
#define L3GD20_FULLSCALE_500       						((uint8_t)0x10)
#define L3GD20_FULLSCALE_2000      						((uint8_t)0x20)
#define L3GD20_FULLSCALE_SELECTION 						((uint8_t)0x30)
#define L3GD20_SPI_3WIRE 								((uint8_t)0x01)

// REG5
#define L3GD20_BOOT_REBOOTMEMORY           				((uint8_t)0x80) // if 0x00 then normal mode
#define L3GD20_FIFO_ENABLE           					((uint8_t)0x40)
#define L3GD20_HIGHPASSFILTER_ENABLE	   				((uint8_t)0x10)
#define L3GD20_INT1SEL_LPF1	   							((uint8_t)0x00)
#define L3GD20_INT1SEL_HPF	   							((uint8_t)0x04)
#define L3GD20_INT1SEL_LPF2	   							((uint8_t)0x08)
#define L3GD20_OUTSEL_LPF1	   							((uint8_t)0x00)
#define L3GD20_OUTSEL_HPF	   							((uint8_t)0x01)
#define L3GD20_OUTSEL_LPF2	   							((uint8_t)0x02)

// FIFO_CTRL_REG
#define L3GD20_FIFO_BYPASS_MODE	   						((uint8_t)0x00)
#define L3GD20_FIFO_MODE	   							((uint8_t)0x20)
#define L3GD20_FIFO_STREAM_MODE	   						((uint8_t)0x40)
#define L3GD20_FIFO_STREAM_TO_FIFO_MODE	   				((uint8_t)0x60)
#define L3GD20_FIFO_BYPASS_TO_STREAM_MODE				((uint8_t)0x80)
#define L3GD20_WATERMARK_LEVEL(x)						((uint8_t)(x & 0x1F))

// INT1_CFG
#define L3GD20_AND_COMBINATION_EVENTS  					((uint8_t)0x80)
#define L3GD20_Z_HIGH_IE  								((uint8_t)0x20)
#define L3GD20_Z_LOW_IE  								((uint8_t)0x10)
#define L3GD20_Y_HIGH_IE  								((uint8_t)0x08)
#define L3GD20_Y_LOW_IE  								((uint8_t)0x04)
#define L3GD20_X_HIGH_IE  								((uint8_t)0x02)
#define L3GD20_X_LOW_IE  								((uint8_t)0x01)

bool l3gd20_make(struct l3gd20_gyro *gyro, spi_device_t *dev, struct gpio_pin *cs);

#ifdef __cplusplus
  }
#endif
  
#endif /* __L3GD20_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
