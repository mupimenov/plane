#include "gpio.h"

#include <stddef.h>

bool
gpio_configure(const struct gpio_pin *gp)
{
	if (gp->configure != NULL)
	{
		return gp->configure(gp);
	}

	return false;
}

bool
gpio_dir(const struct gpio_pin *gp, bool enable_output)
{
	if (gp->dir != NULL)
	{
		return gp->dir(gp, enable_output);
	}

	return false;
}

bool
gpio_set(const struct gpio_pin *gp)
{
	if (gp->set != NULL)
	{
		return gp->set(gp);
	}

	return false;
}

bool
gpio_clear(const struct gpio_pin *gp)
{
	if (gp->clear != NULL)
	{
		return gp->clear(gp);
	}

	return false;
}

int
gpio_pin(const struct gpio_pin *gp)
{
	if (gp->pin != NULL)
	{
		return gp->pin(gp);
	}

	return false;
}
