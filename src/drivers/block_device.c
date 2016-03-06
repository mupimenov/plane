#include "block_device.h"

#include <stddef.h>

bool
device_open(struct block_device *dev)
{
	if (dev->open != NULL)
	{
		return dev->open(dev);
	}

	return true;
}

bool
device_close(struct block_device *dev)
{
	if (dev->close != NULL)
	{
		return dev->close(dev);
	}

	return true;
}

bool
device_setup(struct block_device *dev, unsigned long parameter, unsigned long value)
{
	if (dev->setup != NULL)
	{
		return dev->setup(dev, parameter, value);
	}

	return true;
}

bool
device_configure(struct block_device *dev)
{
	if (dev->configure != NULL)
	{
		return dev->configure(dev);
	}

	return true;
}

int 
device_read(struct block_device *dev, unsigned long address, unsigned char *buffer, unsigned short available_size)
{
	if (dev->read != NULL)
	{
		return dev->read(dev, address, buffer, available_size);
	}

	return 0;
}

int 
device_write(struct block_device *dev, unsigned long address, const unsigned char *buffer, unsigned short size)
{
	if (dev->write != NULL)
	{
		return dev->write(dev, address, buffer, size);
	}

	return 0;
}
