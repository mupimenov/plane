#include "stm32l4_gpio_pin.h"

bool
stm32l4_gpio_configure(const struct gpio_pin *gp)
{	
	const struct stm32l4_gpio_pin *sgp = (const struct stm32l4_gpio_pin*)gp;
	
	GPIO_InitTypeDef gpio_init_struct;
	
	gpio_init_struct.Pin = sgp->pin;
	gpio_init_struct.Mode = sgp->mode;
	gpio_init_struct.Pull = sgp->pull;
	gpio_init_struct.Speed = sgp->speed;
	gpio_init_struct.Alternate = sgp->alternate;
	
	HAL_GPIO_Init(sgp->gpio, &gpio_init_struct);
	
	return true;
}

bool
stm32l4_gpio_dir(const struct gpio_pin *gp, bool enable_output)
{
	const struct stm32l4_gpio_pin *sgp = (const struct stm32l4_gpio_pin*)gp;
	
	GPIO_InitTypeDef gpio_init_struct;
	
	gpio_init_struct.Pin = sgp->pin;
	gpio_init_struct.Mode = enable_output? GPIO_MODE_OUTPUT_PP: GPIO_MODE_INPUT;
	gpio_init_struct.Pull = sgp->pull;
	gpio_init_struct.Speed = sgp->speed;
	gpio_init_struct.Alternate = sgp->alternate;

	HAL_GPIO_Init(sgp->gpio, &gpio_init_struct);
	
	return true;
}

bool
stm32l4_gpio_set(const struct gpio_pin *gp)
{
	const struct stm32l4_gpio_pin *sgp = (const struct stm32l4_gpio_pin*)gp;
	
	HAL_GPIO_WritePin(sgp->gpio, sgp->pin, GPIO_PIN_SET);
	
	return true;
}

bool
stm32l4_gpio_clear(const struct gpio_pin *gp)
{
	const struct stm32l4_gpio_pin *sgp = (const struct stm32l4_gpio_pin*)gp;
	
	HAL_GPIO_WritePin(sgp->gpio, sgp->pin, GPIO_PIN_RESET);
	
	return true;
}

int
stm32l4_gpio_pin(const struct gpio_pin *gp)
{
	const struct stm32l4_gpio_pin *sgp = (const struct stm32l4_gpio_pin*)gp;
	
	return (int)HAL_GPIO_ReadPin(sgp->gpio, sgp->pin);
}
