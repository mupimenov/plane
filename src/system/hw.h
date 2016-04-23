/*
 * hw.h
 *
 *  Created on: Feb 22, 2016
 *      Author: mupimenov
 */

#ifndef SRC_SYSTEM_HW_H_
#define SRC_SYSTEM_HW_H_

#include <stdint.h>

#include "uart_device.h"

#include "gyro.h"
#include "accelerometer.h"
#include "magneto.h"

void hw_led_init(void);
void hw_led_set_red(int state);
void hw_led_set_green(int state);

void hw_link_init(void);
uart_device_t *hw_get_link(void);

void hw_mems_init(void);
struct gyro_driver *hw_get_gyro(void);
struct accelerometer_driver *hw_get_accel(void);
struct magneto_driver *hw_get_mag(void);

struct rtc_date
{
	uint8_t day;
	uint8_t month;
	uint8_t year;
};

struct rtc_time
{
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
};

void hw_rtc_init(void);

void hw_rtc_set_date(struct rtc_date *d);
struct rtc_date hw_rtc_get_date(void);

void hw_rtc_set_time(struct rtc_time *t);
struct rtc_time hw_rtc_get_time(void);

#endif /* SRC_SYSTEM_HW_H_ */
