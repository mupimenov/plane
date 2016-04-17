/*
 * task_measure.h
 *
 *  Created on: Feb 27, 2016
 *      Author: mupimenov
 */

#ifndef SRC_TASKS_TASK_MEASURE_H_
#define SRC_TASKS_TASK_MEASURE_H_

#include <stdint.h>
#include <stdbool.h>

void task_measure_init(void);

void get_gyro_data(float *data);
void get_accel_data(float *data);
void get_mag_data(float *data);
void get_angle_data(float *data);

uint32_t get_measuring_duration(void);

void calibrate(void);
bool calibration_in_progress(void);

#endif /* SRC_TASKS_TASK_MEASURE_H_ */
