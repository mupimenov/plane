/*
 * accelerometer.h
 *
 *  Created on: Mar 5, 2016
 *      Author: mupimenov
 */

#ifndef SRC_DRIVERS_ACCELEROMETER_H_
#define SRC_DRIVERS_ACCELEROMETER_H_

#include <stdbool.h>

struct accelerometer_driver
{
  bool (*open)(struct accelerometer_driver *);
  bool (*configure)(struct accelerometer_driver *);
  bool (*read)(struct accelerometer_driver *, float *);
  bool (*close)(struct accelerometer_driver *);
};

bool accelerometer_open(struct accelerometer_driver *accel);
bool accelerometer_configure(struct accelerometer_driver *accel);
bool accelerometer_read(struct accelerometer_driver *accel, float *data);
bool accelerometer_close(struct accelerometer_driver *accel);

#endif /* SRC_DRIVERS_ACCELEROMETER_H_ */
