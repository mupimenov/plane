/*
 * magneto.c
 *
 *  Created on: Mar 12, 2016
 *      Author: mupimenov
 */

#include "magneto.h"

#include <stddef.h>

bool magneto_open(struct magneto_driver *mag)
{
	if (mag->open != NULL)
		return mag->open(mag);
	return true;
}

bool magneto_configure(struct magneto_driver *mag)
{
	if (mag->configure != NULL)
		return mag->configure(mag);
	return true;
}

bool magneto_read(struct magneto_driver *mag, float *data)
{
	if (mag->read != NULL)
		return mag->read(mag, data);
	return true;
}

bool magneto_close(struct magneto_driver *mag)
{
	if (mag->close != NULL)
		return mag->close(mag);
	return true;
}
