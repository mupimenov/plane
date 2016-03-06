/*
 * gyro.c
 *
 *  Created on: Feb 23, 2016
 *      Author: mupimenov
 */

#include "gyro.h"

#include <stddef.h>

bool gyro_open(struct gyro_driver *gyro)
{
	if (gyro->open != NULL)
		return gyro->open(gyro);
	return true;
}

bool gyro_configure(struct gyro_driver *gyro)
{
	if (gyro->configure != NULL)
		return gyro->configure(gyro);
	return true;
}

bool gyro_read(struct gyro_driver *gyro, float *data)
{
	if (gyro->read != NULL)
		return gyro->read(gyro, data);
	return true;
}

bool gyro_close(struct gyro_driver *gyro)
{
	if (gyro->close != NULL)
		return gyro->close(gyro);
	return true;
}
