/*
 * madgwick_ahrs.c
 *
 *  Created on: Mar 5, 2016
 *      Author: mupimenov
 */

#include "madgwick_ahrs.h"

#include <math.h>

void madgwick_ahrs_init(struct madgwick_ahrs_instance *instance, float period, float beta)
{
	instance->period = period;
	instance->beta = beta;

	instance->q0 = 1.0f;
	instance->q1 = 0.0f;
	instance->q2 = 0.0f;
	instance->q3 = 0.0f;
}

static float _madgwick_ahrs_inv_sqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void madgwick_ahrs_update_imu(struct madgwick_ahrs_instance *instance,
		float gx, float gy, float gz,
		float ax, float ay, float az)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-instance->q1 * gx - instance->q2 * gy - instance->q3 * gz);
	qDot2 = 0.5f * (instance->q0 * gx + instance->q2 * gz - instance->q3 * gy);
	qDot3 = 0.5f * (instance->q0 * gy - instance->q1 * gz + instance->q3 * gx);
	qDot4 = 0.5f * (instance->q0 * gz + instance->q1 * gy - instance->q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = _madgwick_ahrs_inv_sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * instance->q0;
		_2q1 = 2.0f * instance->q1;
		_2q2 = 2.0f * instance->q2;
		_2q3 = 2.0f * instance->q3;
		_4q0 = 4.0f * instance->q0;
		_4q1 = 4.0f * instance->q1;
		_4q2 = 4.0f * instance->q2;
		_8q1 = 8.0f * instance->q1;
		_8q2 = 8.0f * instance->q2;
		q0q0 = instance->q0 * instance->q0;
		q1q1 = instance->q1 * instance->q1;
		q2q2 = instance->q2 * instance->q2;
		q3q3 = instance->q3 * instance->q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * instance->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * instance->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * instance->q3 - _2q1 * ax + 4.0f * q2q2 * instance->q3 - _2q2 * ay;
		recipNorm = _madgwick_ahrs_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= instance->beta * s0;
		qDot2 -= instance->beta * s1;
		qDot3 -= instance->beta * s2;
		qDot4 -= instance->beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	instance->q0 += qDot1 * instance->period;
	instance->q1 += qDot2 * instance->period;
	instance->q2 += qDot3 * instance->period;
	instance->q3 += qDot4 * instance->period;

	// Normalise quaternion
	recipNorm = _madgwick_ahrs_inv_sqrt(instance->q0 * instance->q0 +
			instance->q1 * instance->q1 + instance->q2 * instance->q2 +
			instance->q3 * instance->q3);
	instance->q0 *= recipNorm;
	instance->q1 *= recipNorm;
	instance->q2 *= recipNorm;
	instance->q3 *= recipNorm;
}

void madgwick_ahrs_update(struct madgwick_ahrs_instance *instance,
		float gx, float gy, float gz,
		float ax, float ay, float az,
		float mx, float my, float mz)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-instance->q1 * gx - instance->q2 * gy - instance->q3 * gz);
	qDot2 = 0.5f * (instance->q0 * gx + instance->q2 * gz - instance->q3 * gy);
	qDot3 = 0.5f * (instance->q0 * gy - instance->q1 * gz + instance->q3 * gx);
	qDot4 = 0.5f * (instance->q0 * gz + instance->q1 * gy - instance->q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = _madgwick_ahrs_inv_sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = _madgwick_ahrs_inv_sqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * instance->q0 * mx;
		_2q0my = 2.0f * instance->q0 * my;
		_2q0mz = 2.0f * instance->q0 * mz;
		_2q1mx = 2.0f * instance->q1 * mx;
		_2q0 = 2.0f * instance->q0;
		_2q1 = 2.0f * instance->q1;
		_2q2 = 2.0f * instance->q2;
		_2q3 = 2.0f * instance->q3;
		_2q0q2 = 2.0f * instance->q0 * instance->q2;
		_2q2q3 = 2.0f * instance->q2 * instance->q3;
		q0q0 = instance->q0 * instance->q0;
		q0q1 = instance->q0 * instance->q1;
		q0q2 = instance->q0 * instance->q2;
		q0q3 = instance->q0 * instance->q3;
		q1q1 = instance->q1 * instance->q1;
		q1q2 = instance->q1 * instance->q2;
		q1q3 = instance->q1 * instance->q3;
		q2q2 = instance->q2 * instance->q2;
		q2q3 = instance->q2 * instance->q3;
		q3q3 = instance->q3 * instance->q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * instance->q3 + _2q0mz * instance->q2 + mx * q1q1 + _2q1 * my * instance->q2 + _2q1 * mz * instance->q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * instance->q3 + my * q0q0 - _2q0mz * instance->q1 + _2q1mx * instance->q2 - my * q1q1 + my * q2q2 + _2q2 * mz * instance->q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * instance->q2 + _2q0my * instance->q1 + mz * q0q0 + _2q1mx * instance->q3 - mz * q1q1 + _2q2 * my * instance->q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * instance->q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * instance->q3 + _2bz * instance->q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * instance->q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * instance->q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * instance->q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * instance->q2 + _2bz * instance->q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * instance->q3 - _4bz * instance->q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * instance->q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * instance->q2 - _2bz * instance->q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * instance->q1 + _2bz * instance->q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * instance->q0 - _4bz * instance->q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * instance->q3 + _2bz * instance->q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * instance->q0 + _2bz * instance->q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * instance->q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = _madgwick_ahrs_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= instance->beta * s0;
		qDot2 -= instance->beta * s1;
		qDot3 -= instance->beta * s2;
		qDot4 -= instance->beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	instance->q0 += qDot1 * instance->period;
	instance->q1 += qDot2 * instance->period;
	instance->q2 += qDot3 * instance->period;
	instance->q3 += qDot4 * instance->period;

	// Normalise quaternion
	recipNorm = _madgwick_ahrs_inv_sqrt(instance->q0 * instance->q0 + instance->q1 * instance->q1 + instance->q2 * instance->q2 + instance->q3 * instance->q3);
	instance->q0 *= recipNorm;
	instance->q1 *= recipNorm;
	instance->q2 *= recipNorm;
	instance->q3 *= recipNorm;
}

float madgwick_ahrs_get_pitch(struct madgwick_ahrs_instance *instance)
{
	return atan2(2 * instance->q2 * instance->q3 - 2 * instance->q0 * instance->q1,
			2 * instance->q0 * instance->q0 + 2 * instance->q3 * instance->q3 - 1);
}

float madgwick_ahrs_get_roll(struct madgwick_ahrs_instance *instance)
{
	return -1 * asin(2 * instance->q1 * instance->q3 + 2 * instance->q0 * instance->q2);
}

float madgwick_ahrs_get_yaw(struct madgwick_ahrs_instance *instance)
{
	return atan2(2 * instance->q1 * instance->q2 - 2 * instance->q0 * instance->q3,
			2 * instance->q0 * instance->q0 + 2 * instance->q1 * instance->q1 - 1);
}
