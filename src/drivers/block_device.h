#ifndef __BLOCK_DEVICE_H
#define __BLOCK_DEVICE_H

#include <stdbool.h>

struct block_device
{
	bool (*open)(struct block_device *dev);
	bool (*close)(struct block_device *dev);

	bool (*setup)(struct block_device *dev, unsigned long parameter, unsigned long value);
	bool (*configure)(struct block_device *dev);
	int (*read)(struct block_device *dev, unsigned long address, unsigned char *buffer, unsigned short available_size);
	int (*write)(struct block_device *dev, unsigned long address, const unsigned char *buffer, unsigned short size);
};

#define device_set_open(bd,fn) (bd).open = fn
#define device_set_close(bd,fn) (bd).close = fn

bool
block_open(struct block_device *dev);

bool
block_close(struct block_device *dev);

bool
block_setup(struct block_device *dev, unsigned long parameter, unsigned long value);

bool
block_configure(struct block_device *dev);

int 
block_read(struct block_device *dev, unsigned long address, unsigned char *buffer, unsigned short available_size);

int 
block_write(struct block_device *dev, unsigned long address, const unsigned char *buffer, unsigned short size);

#endif
