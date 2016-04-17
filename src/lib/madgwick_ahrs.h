/*
 * madgwick_ahrs.h
 *
 *  Created on: Mar 5, 2016
 *      Author: mupimenov
 */

#ifndef SRC_LIB_MADGWICK_AHRS_H_
#define SRC_LIB_MADGWICK_AHRS_H_

struct madgwick_ahrs_instance {
	volatile float period;
	volatile float beta;

	volatile float q0;
	volatile float q1;
	volatile float q2;
	volatile float q3;
};

void madgwick_ahrs_init(struct madgwick_ahrs_instance *instance, float period, float beta);

void madgwick_ahrs_update_imu(struct madgwick_ahrs_instance *instance, float gx, float gy, float gz, float ax, float ay, float az);
void madgwick_ahrs_update(struct madgwick_ahrs_instance *instance, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

float madgwick_ahrs_get_pitch(struct madgwick_ahrs_instance *instance);
float madgwick_ahrs_get_roll(struct madgwick_ahrs_instance *instance);
float madgwick_ahrs_get_yaw(struct madgwick_ahrs_instance *instance);

#endif /* SRC_LIB_MADGWICK_AHRS_H_ */
