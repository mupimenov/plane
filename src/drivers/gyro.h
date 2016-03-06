#ifndef __GYRO_H
#define __GYRO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>

struct gyro_driver
{  
  bool (*open)(struct gyro_driver *);
  bool (*configure)(struct gyro_driver *);
  bool (*read)(struct gyro_driver *, float *);
  bool (*close)(struct gyro_driver *);
};

bool gyro_open(struct gyro_driver *gyro);
bool gyro_configure(struct gyro_driver *gyro);
bool gyro_read(struct gyro_driver *gyro, float *data);
bool gyro_close(struct gyro_driver *gyro);

#ifdef __cplusplus
}
#endif

#endif /* __GYRO_H */
