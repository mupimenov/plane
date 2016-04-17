/*
 * char_device.h
 *
 *  Created on: Mar 12, 2016
 *      Author: mupimenov
 */

#ifndef __CHAR_DEVICE_H
#define __CHAR_DEVICE_H

#include <stdbool.h>

struct char_device
{
	bool (*open)(struct char_device *dev);
	bool (*close)(struct char_device *dev);

	bool (*setup)(struct char_device *dev, unsigned long parameter, unsigned long value);
	bool (*configure)(struct char_device *dev);
	int (*read)(struct char_device *dev, unsigned char *buffer, unsigned short available_size);
	int (*write)(struct char_device *dev, const unsigned char *buffer, unsigned short size);
};

#define device_set_open(bd,fn) (bd).open = fn
#define device_set_close(bd,fn) (bd).close = fn

bool
device_open(struct char_device *dev);

bool
device_close(struct char_device *dev);

bool
device_setup(struct char_device *dev, unsigned long parameter, unsigned long value);

bool
device_configure(struct char_device *dev);

int
device_read(struct char_device *dev, unsigned char *buffer, unsigned short available_size);

int
device_write(struct char_device *dev, const unsigned char *buffer, unsigned short size);

#endif
