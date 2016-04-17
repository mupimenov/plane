/*
 * magneto.h
 *
 *  Created on: Mar 12, 2016
 *      Author: mupimenov
 */

#ifndef SRC_DRIVERS_MAGNETO_H_
#define SRC_DRIVERS_MAGNETO_H_

#include <stdbool.h>

struct magneto_driver
{
  bool (*open)(struct magneto_driver *);
  bool (*configure)(struct magneto_driver *);
  bool (*read)(struct magneto_driver *, float *);
  bool (*close)(struct magneto_driver *);
};

bool magneto_open(struct magneto_driver *mag);
bool magneto_configure(struct magneto_driver *mag);
bool magneto_read(struct magneto_driver *mag, float *data);
bool magneto_close(struct magneto_driver *mag);

#endif /* SRC_DRIVERS_MAGNETO_H_ */
