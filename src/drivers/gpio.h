#ifndef __GPIO_H
#define __GPIO_H

#include <stdbool.h>

struct gpio_pin
{
	bool (*configure)(const struct gpio_pin *gp);
	bool (*dir)(const struct gpio_pin *gp, bool enable_output);
	bool (*set)(const struct gpio_pin *gp);
	bool (*clear)(const struct gpio_pin *gp);
	int (*pin)(const struct gpio_pin *gp);
};

bool
gpio_configure(const struct gpio_pin *gp);

bool
gpio_dir(const struct gpio_pin *gp, bool enable_output);

bool
gpio_set(const struct gpio_pin *gp);

bool
gpio_clear(const struct gpio_pin *gp);

int
gpio_pin(const struct gpio_pin *gp);

#endif
