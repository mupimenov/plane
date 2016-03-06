#ifndef __STM32L4_GPIO_PIN_H
#define __STM32L4_GPIO_PIN_H

#include "gpio.h"

#include "stm32l4xx_hal.h"

struct stm32l4_gpio_pin
{
	struct gpio_pin interface;
	
	GPIO_TypeDef *gpio;
	unsigned short pin;
	unsigned long mode;
	unsigned char pull;
	unsigned char speed;
	unsigned char alternate;
};

bool
stm32l4_gpio_configure(const struct gpio_pin *gp);

bool
stm32l4_gpio_dir(const struct gpio_pin *gp, bool enable_output);

bool
stm32l4_gpio_set(const struct gpio_pin *gp);

bool
stm32l4_gpio_clear(const struct gpio_pin *gp);

int
stm32l4_gpio_pin(const struct gpio_pin *gp);

#define GPIO_INSTANCE(gpio,pin,mode,pull,speed,alternate) {\
{stm32l4_gpio_configure, stm32l4_gpio_dir, stm32l4_gpio_set, stm32l4_gpio_clear, stm32l4_gpio_pin},\
gpio, pin, mode, pull, speed, alternate \
}

#define GPIO_FAKE() {\
{NULL, NULL, NULL, NULL, NULL},\
0, 0, 0, 0, 0, 0 \
}

#endif
