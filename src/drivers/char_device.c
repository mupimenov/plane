/*
 * char_device.c
 *
 *  Created on: Mar 12, 2016
 *      Author: mupimenov
 */

#include "char_device.h"

#include <stddef.h>

bool
device_open(struct char_device *dev)
{
	if (dev->open != NULL)
	{
		return dev->open(dev);
	}

	return true;
}

bool
device_close(struct char_device *dev)
{
	if (dev->close != NULL)
	{
		return dev->close(dev);
	}

	return true;
}

bool
device_setup(struct char_device *dev, unsigned long parameter, unsigned long value)
{
	if (dev->setup != NULL)
	{
		return dev->setup(dev, parameter, value);
	}

	return true;
}

bool
device_configure(struct char_device *dev)
{
	if (dev->configure != NULL)
	{
		return dev->configure(dev);
	}

	return true;
}

int
device_read(struct char_device *dev, unsigned char *buffer, unsigned short available_size)
{
	if (dev->read != NULL)
	{
		return dev->read(dev, buffer, available_size);
	}

	return 0;
}

int
device_write(struct char_device *dev, const unsigned char *buffer, unsigned short size)
{
	if (dev->write != NULL)
	{
		return dev->write(dev, buffer, size);
	}

	return 0;
}
