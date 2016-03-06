/*
 * accelerometer.c
 *
 *  Created on: Mar 5, 2016
 *      Author: mupimenov
 */

#include "accelerometer.h"

#include <stddef.h>

bool accelerometer_open(struct accelerometer_driver *accel)
{
	if (accel->open != NULL)
		return accel->open(accel);
	return true;
}

bool accelerometer_configure(struct accelerometer_driver *accel)
{
	if (accel->configure != NULL)
		return accel->configure(accel);
	return true;
}

bool accelerometer_read(struct accelerometer_driver *accel, float *data)
{
	if (accel->read != NULL)
		return accel->read(accel, data);
	return true;
}

bool accelerometer_close(struct accelerometer_driver *accel)
{
	if (accel->close != NULL)
		return accel->close(accel);
	return true;
}
