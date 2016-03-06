TARGET = plane
VERSION = 0.9.0

CC = /home/mupimenov/Programs/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc
CCFLAGS = -Wall -O0 -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -g -g3 -fsingle-precision-constant -mfpu=fpv4-sp-d16
#-mfpu=fpv4-sp-d16 -nostartfiles
INCLUDE = -Isrc/config -Isrc/drivers -Isrc/system -Isrc/freertos/include -Isrc/freertos/portable -Isrc/lib -Isrc/os -Isrc/system/hal/include -Isrc/tasks
DEFINES = -DVERSION=\"$(VERSION)\" -DSTM32L476xx -DUSE_FULL_LL_DRIVER

SRCS = src/drivers/accelerometer.c \
	src/drivers/block_device.c \
	src/drivers/gpio.c \
	src/drivers/gyro.c \
	src/drivers/l3gd20.c \
	src/drivers/lsm303c.c \
	src/drivers/stm32l4_gpio_pin.c \
	src/drivers/stm32l4_spi_device.c \
	src/drivers/stm32l4_uart_device.c \
	src/freertos/portable/port.c \
	src/freertos/croutine.c \
	src/freertos/event_groups.c \
	src/freertos/heap_3_fixed.c \
	src/freertos/list.c \
	src/freertos/queue.c \
	src/freertos/tasks.c \
	src/freertos/timers.c \
	src/lib/filters.c \
	src/lib/kalman.c \
	src/lib/madgwick_ahrs.c \
	src/lib/modbus.c \
	src/os/cmsis_os.c \
	src/system/hal/src/stm32l4xx_hal_cortex.c \
	src/system/hal/src/stm32l4xx_hal_dma.c \
	src/system/hal/src/stm32l4xx_hal_flash_ex.c \
	src/system/hal/src/stm32l4xx_hal_flash_ramfunc.c \
	src/system/hal/src/stm32l4xx_hal_flash.c \
	src/system/hal/src/stm32l4xx_hal_gpio.c \
	src/system/hal/src/stm32l4xx_hal_i2c_ex.c \
	src/system/hal/src/stm32l4xx_hal_i2c.c \
	src/system/hal/src/stm32l4xx_hal_pcd_ex.c \
	src/system/hal/src/stm32l4xx_hal_pcd.c \
	src/system/hal/src/stm32l4xx_hal_pwr_ex.c \
	src/system/hal/src/stm32l4xx_hal_pwr.c \
	src/system/hal/src/stm32l4xx_hal_qspi.c \
	src/system/hal/src/stm32l4xx_hal_rcc_ex.c \
	src/system/hal/src/stm32l4xx_hal_rcc.c \
	src/system/hal/src/stm32l4xx_hal_rtc_ex.c \
	src/system/hal/src/stm32l4xx_hal_rtc.c \
	src/system/hal/src/stm32l4xx_hal_tim_ex.c \
	src/system/hal/src/stm32l4xx_hal_tim.c \
	src/system/hal/src/stm32l4xx_hal.c \
	src/system/hal/src/stm32l4xx_ll_dma.c \
	src/system/hal/src/stm32l4xx_ll_rcc.c \
	src/system/hal/src/stm32l4xx_ll_spi.c \
	src/system/hal/src/stm32l4xx_ll_usart.c \
	src/system/hal/src/stm32l4xx_ll_usb.c \
	src/system/cmsis_nvic.c \
	src/system/hw.c \
	src/system/stm32l4xx_hal_msp.c \
	src/system/stm32l4xx_it.c \
	src/system/system_stm32l4xx.c \
	src/tasks/task_led.c \
	src/tasks/task_link.c \
	src/tasks/task_measure.c \
	src/main.c
	
OBJS = $(subst .c,.o,$(SRCS)) src/system/startup_stm32l476xx.o

LIBS = -lm 
LDLAGS  = -Tsrc/system/STM32L476XX.ld -nostartfiles -mthumb -mcpu=cortex-m4 -u _printf_float -u _scanf_float 
# -L/home/mupimenov/Programs/gcc-arm-none-eabi-4_9-2015q3/arm-none-eabi/lib/thumb 
# 

$(TARGET):	$(OBJS)
	$(CC) $(LDLAGS) -o $(TARGET) $(OBJS) $(LIBS) -Wl,-Map=$(TARGET).map

%.o: %.c
	$(CC) $(CCFLAGS) $(INCLUDE) $(DEFINES) -c -o $@ $<
	
src/system/startup_stm32l476xx.o:
	$(CC) $(CCFLAGS) $(INCLUDE) $(DEFINES) -c -o $@ src/system/startup_stm32l476xx.s

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
	