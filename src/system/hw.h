/*
 * hw.h
 *
 *  Created on: Feb 22, 2016
 *      Author: mupimenov
 */

#ifndef SRC_SYSTEM_HW_H_
#define SRC_SYSTEM_HW_H_

#include "uart_device.h"
#include "gyro.h"
#include "accelerometer.h"

void hw_led_init(void);
void hw_led_set_red(int state);
void hw_led_set_green(int state);

void hw_link_init(void);
uart_device_t *hw_get_link(void);

void hw_mems_init(void);
struct gyro_driver *hw_get_gyro(void);
struct accelerometer_driver *hw_get_accel(void);

#endif /* SRC_SYSTEM_HW_H_ */
