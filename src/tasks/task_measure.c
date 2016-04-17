/*
 * task_measure.c
 *
 *  Created on: Feb 27, 2016
 *      Author: mupimenov
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "cmsis_os.h"
#include "hw.h"

#include "kalman.h"
#include "filters.h"
#include "madgwick_ahrs.h"

#define GYRO_KALMAN_ENABLE 		0
#define ACCEL_KALMAN_ENABLE 	0

#define MEASURE_TASK_PERIOD_MS 	10

static osThreadId task_measure;
static osMutexId msr_mutex;

enum measure_state {
	MEASURE_NORMAL,
	MEASURE_CALIBRATION
};

static enum measure_state 	msr_state = MEASURE_CALIBRATION;

/* GYRO */

static float msr_raw_gyro_data[3] = { 0.0f };
static float msr_filtered_gyro_data[3] = { 0.0f };
static float msr_angle_data[3] = { 0.0f };

static bool msr_gyro_filter_enable = false;

#if GYRO_KALMAN_ENABLE

#define GYRO_KALMAN_FILTER_F (1.0f)
#define GYRO_KALMAN_FILTER_B 0.0f
#define GYRO_KALMAN_FILTER_H 1.0f
#define GYRO_KALMAN_FILTER_I 1.0f
#define GYRO_KALMAN_FILTER_Q 0.5f
#define GYRO_KALMAN_FILTER_R 1.5f

#define GYRO_KALMAN_FILTER_P_INIT 0.2f

static struct kalman1_params msr_gyro_x_kalman_params;
static struct kalman1_state msr_gyro_x_kalman_state;
static struct kalman1_params msr_gyro_y_kalman_params;
static struct kalman1_state msr_gyro_y_kalman_state;
static struct kalman1_params msr_gyro_z_kalman_params;
static struct kalman1_state msr_gyro_z_kalman_state;

#else

// Fc=10 Hz, Td=10 msec
#define GYRO_FILTER_B1 (0.24492f)
#define GYRO_FILTER_A1 (-1.15805f)
#define GYRO_FILTER_A2 (0.4032f)

static const struct filter2z_params msr_gyro_x_filter2z_params = {
		GYRO_FILTER_B1,
		GYRO_FILTER_A1, GYRO_FILTER_A2
};

static struct filter2z_state 		msr_gyro_x_filter2z_state;

static const struct filter2z_params msr_gyro_y_filter2z_params = {
		GYRO_FILTER_B1,
		GYRO_FILTER_A1, GYRO_FILTER_A2
};

static struct filter2z_state 		msr_gyro_y_filter2z_state;

static const struct filter2z_params msr_gyro_z_filter2z_params = {
		GYRO_FILTER_B1,
		GYRO_FILTER_A1, GYRO_FILTER_A2
};

static struct filter2z_state 		msr_gyro_z_filter2z_state;

#endif

/* ACCEL */

static float msr_raw_accel_data[3] = { 0.0f };
static float msr_filtered_accel_data[3] = { 0.0f };

static bool msr_accel_filter_enable = false;

#if ACCEL_KALMAN_ENABLE

#define ACCEL_KALMAN_FILTER_F (1.0f)
#define ACCEL_KALMAN_FILTER_B 0.0f
#define ACCEL_KALMAN_FILTER_H 1.0f
#define ACCEL_KALMAN_FILTER_I 1.0f
#define ACCEL_KALMAN_FILTER_Q 0.1f
#define ACCEL_KALMAN_FILTER_R 0.2f

#define ACCEL_KALMAN_FILTER_P_INIT 0.2f

static struct kalman1_params msr_accel_x_kalman_params;
static struct kalman1_state msr_accel_x_kalman_state;
static struct kalman1_params msr_accel_y_kalman_params;
static struct kalman1_state msr_accel_y_kalman_state;
static struct kalman1_params msr_accel_z_kalman_params;
static struct kalman1_state msr_accel_z_kalman_state;

#else

// Fc=10 Hz, Td=10 msec
#define ACCEL_FILTER_B1 (0.24492f)
#define ACCEL_FILTER_A1 (-1.15805f)
#define ACCEL_FILTER_A2 (0.4032f)

static const struct filter2z_params msr_accel_x_filter2z_params = {
		ACCEL_FILTER_B1,
		ACCEL_FILTER_A1, ACCEL_FILTER_A2
};

static struct filter2z_state 		msr_accel_x_filter2z_state;

static const struct filter2z_params msr_accel_y_filter2z_params = {
		ACCEL_FILTER_B1,
		ACCEL_FILTER_A1, ACCEL_FILTER_A2
};

static struct filter2z_state 		msr_accel_y_filter2z_state;

static const struct filter2z_params msr_accel_z_filter2z_params = {
		ACCEL_FILTER_B1,
		ACCEL_FILTER_A1, ACCEL_FILTER_A2
};

static struct filter2z_state 		msr_accel_z_filter2z_state;

#endif

/* MAG */

static float msr_raw_mag_data[3] = { 0.0f };
static float msr_filtered_mag_data[3] = { 0.0f };

static bool msr_mag_filter_enable = false;

// Fc=2 Hz, Td=100 msec
#define MAG_FILTER_B1 (0.56726f)
#define MAG_FILTER_A1 (-0.51859f)
#define MAG_FILTER_A2 (0.08912f)

static const struct filter2z_params msr_mag_x_filter2z_params = {
		MAG_FILTER_B1,
		MAG_FILTER_A1, MAG_FILTER_A2
};

static struct filter2z_state 		msr_mag_x_filter2z_state;

static const struct filter2z_params msr_mag_y_filter2z_params = {
		MAG_FILTER_B1,
		MAG_FILTER_A1, MAG_FILTER_A2
};

static struct filter2z_state 		msr_mag_y_filter2z_state;

static const struct filter2z_params msr_mag_z_filter2z_params = {
		MAG_FILTER_B1,
		MAG_FILTER_A1, MAG_FILTER_A2
};

static struct filter2z_state 		msr_mag_z_filter2z_state;

/* AHRS */

static struct madgwick_ahrs_instance msr_madgwick_ahrs;

/* CALIBRATION */
static struct filter2z_state 		msr_calib_accel_x_filter2z_state;
static struct filter2z_state 		msr_calib_accel_y_filter2z_state;
static struct filter2z_state 		msr_calib_accel_z_filter2z_state;

static float msr_accel_zero_level[3] = { 0.0f };


static uint32_t 					msr_measuring_duration;

static void _get_gyro_data(struct gyro_driver *gyro)
{
	do
	{
		if (!gyro_open(gyro))
			break;
		if (!gyro_read(gyro, msr_raw_gyro_data))
			break;

		if (msr_gyro_filter_enable)
		{
#if GYRO_KALMAN_ENABLE
			kalman1_correct(&msr_gyro_x_kalman_params, &msr_gyro_x_kalman_state, msr_raw_gyro_data[0], 0.0f);
			kalman1_correct(&msr_gyro_y_kalman_params, &msr_gyro_y_kalman_state, msr_raw_gyro_data[1], 0.0f);
			kalman1_correct(&msr_gyro_z_kalman_params, &msr_gyro_z_kalman_state, msr_raw_gyro_data[2], 0.0f);

			osMutexWait(msr_mutex, osWaitForever);

			msr_filtered_gyro_data[0] = msr_gyro_x_kalman_state.x_previous;
			msr_filtered_gyro_data[1] = msr_gyro_y_kalman_state.x_previous;
			msr_filtered_gyro_data[2] = msr_gyro_z_kalman_state.x_previous;

			osMutexRelease(msr_mutex);
#else
			filter2z_update(&msr_gyro_x_filter2z_params, &msr_gyro_x_filter2z_state, msr_raw_gyro_data[0]);
			filter2z_update(&msr_gyro_y_filter2z_params, &msr_gyro_y_filter2z_state, msr_raw_gyro_data[1]);
			filter2z_update(&msr_gyro_z_filter2z_params, &msr_gyro_z_filter2z_state, msr_raw_gyro_data[2]);

			osMutexWait(msr_mutex, osWaitForever);

			msr_filtered_gyro_data[0] = msr_gyro_x_filter2z_state.y;
			msr_filtered_gyro_data[1] = msr_gyro_y_filter2z_state.y;
			msr_filtered_gyro_data[2] = msr_gyro_z_filter2z_state.y;

			osMutexRelease(msr_mutex);
#endif
		}
		else
		{
			osMutexWait(msr_mutex, osWaitForever);

			msr_filtered_gyro_data[0] = msr_raw_gyro_data[0];
			msr_filtered_gyro_data[1] = msr_raw_gyro_data[1];
			msr_filtered_gyro_data[2] = msr_raw_gyro_data[2];

			osMutexRelease(msr_mutex);
		}
	} while (0);

	(void)gyro_close(gyro);
}

static void _get_accel_data(struct accelerometer_driver *accel)
{
	do
	{
		if (!accelerometer_open(accel))
			break;
		if (!accelerometer_read(accel, msr_raw_accel_data))
			break;

		if (msr_accel_filter_enable)
		{
#if ACCEL_KALMAN_ENABLE
			kalman1_correct(&msr_accel_x_kalman_params, &msr_accel_x_kalman_state, msr_raw_accel_data[0], 0.0f);
			kalman1_correct(&msr_accel_y_kalman_params, &msr_accel_y_kalman_state, msr_raw_accel_data[1], 0.0f);
			kalman1_correct(&msr_accel_z_kalman_params, &msr_accel_z_kalman_state, msr_raw_accel_data[2], 0.0f);

			osMutexWait(msr_mutex, osWaitForever);

			msr_filtered_accel_data[0] = msr_accel_x_kalman_state.x_previous;
			msr_filtered_accel_data[1] = msr_accel_y_kalman_state.x_previous;
			msr_filtered_accel_data[2] = msr_accel_z_kalman_state.x_previous;

			osMutexRelease(msr_mutex);
#else
			filter2z_update(&msr_accel_x_filter2z_params, &msr_accel_x_filter2z_state, msr_raw_accel_data[0]);
			filter2z_update(&msr_accel_y_filter2z_params, &msr_accel_y_filter2z_state, msr_raw_accel_data[1]);
			filter2z_update(&msr_accel_z_filter2z_params, &msr_accel_z_filter2z_state, msr_raw_accel_data[2]);

			osMutexWait(msr_mutex, osWaitForever);

			msr_filtered_accel_data[0] = msr_accel_x_filter2z_state.y + msr_accel_zero_level[0];
			msr_filtered_accel_data[1] = msr_accel_y_filter2z_state.y + msr_accel_zero_level[1];
			msr_filtered_accel_data[2] = msr_accel_z_filter2z_state.y + msr_accel_zero_level[2];

			osMutexRelease(msr_mutex);
#endif
		}
		else
		{
			osMutexWait(msr_mutex, osWaitForever);

			msr_filtered_accel_data[0] = msr_raw_accel_data[0] + msr_accel_zero_level[0];
			msr_filtered_accel_data[1] = msr_raw_accel_data[1] + msr_accel_zero_level[1];
			msr_filtered_accel_data[2] = msr_raw_accel_data[2] + msr_accel_zero_level[2];

			osMutexRelease(msr_mutex);
		}
	} while (0);

	(void)accelerometer_close(accel);
}

static void _get_mag_data(struct magneto_driver *mag)
{
	static int step = 0;
	const int bypass_count = 10;

	if (step < bypass_count)
	{
		++step;
		return;
	}

	step = 0;

	do
	{
		if (!magneto_open(mag))
			break;
		if (!magneto_read(mag, msr_raw_mag_data))
			break;

		if (msr_accel_filter_enable)
		{
			filter2z_update(&msr_mag_x_filter2z_params, &msr_mag_x_filter2z_state, msr_raw_mag_data[0]);
			filter2z_update(&msr_mag_y_filter2z_params, &msr_mag_y_filter2z_state, msr_raw_mag_data[1]);
			filter2z_update(&msr_mag_z_filter2z_params, &msr_mag_z_filter2z_state, msr_raw_mag_data[2]);

			osMutexWait(msr_mutex, osWaitForever);

			msr_filtered_mag_data[0] = msr_raw_mag_data[0];
			msr_filtered_mag_data[1] = msr_raw_mag_data[1];
			msr_filtered_mag_data[2] = msr_raw_mag_data[2];

			osMutexRelease(msr_mutex);
		}
		else
		{
			osMutexWait(msr_mutex, osWaitForever);

			msr_filtered_mag_data[0] = msr_raw_mag_data[0];
			msr_filtered_mag_data[1] = msr_raw_mag_data[1];
			msr_filtered_mag_data[2] = msr_raw_mag_data[2];

			osMutexRelease(msr_mutex);
		}

	} while (0);

	(void)magneto_close(mag);
}

static void _get_angle_data(void)
{
	const float deg2rad = (M_PI / 180.0);
	float data[3];
#if 0
	madgwick_ahrs_update_imu(&msr_madgwick_ahrs,
			msr_filtered_gyro_data[0] * deg2rad, msr_filtered_gyro_data[1] * deg2rad, msr_filtered_gyro_data[2] * deg2rad,
			-msr_filtered_accel_data[0], -msr_filtered_accel_data[1], msr_filtered_accel_data[2]);
#else
	madgwick_ahrs_update(&msr_madgwick_ahrs,
			msr_filtered_gyro_data[0] * deg2rad, msr_filtered_gyro_data[1] * deg2rad, msr_filtered_gyro_data[2] * deg2rad,
			-msr_filtered_accel_data[0], -msr_filtered_accel_data[1], msr_filtered_accel_data[2],
			-msr_filtered_mag_data[0], -msr_filtered_mag_data[1], msr_filtered_mag_data[2]);
#endif

	data[0] = madgwick_ahrs_get_pitch(&msr_madgwick_ahrs);
	data[1] = madgwick_ahrs_get_roll(&msr_madgwick_ahrs);
	data[2] = madgwick_ahrs_get_yaw(&msr_madgwick_ahrs);

	osMutexWait(msr_mutex, osWaitForever);

	msr_angle_data[0] = data[0];
	msr_angle_data[1] = data[1];
	msr_angle_data[2] = data[2];

	osMutexRelease(msr_mutex);
}

static void _init_filters(void)
{
	// GYRO

	msr_gyro_filter_enable = true;

#if GYRO_KALMAN_ENABLE
	// X
	msr_gyro_x_kalman_params.f = GYRO_KALMAN_FILTER_F;
	msr_gyro_x_kalman_params.b = GYRO_KALMAN_FILTER_B;
	msr_gyro_x_kalman_params.h = GYRO_KALMAN_FILTER_H;
	msr_gyro_x_kalman_params.i = GYRO_KALMAN_FILTER_I;
	msr_gyro_x_kalman_params.q = GYRO_KALMAN_FILTER_Q;
	msr_gyro_x_kalman_params.r = GYRO_KALMAN_FILTER_R;

	msr_gyro_x_kalman_state.x_previous = 0.0f;
	msr_gyro_x_kalman_state.u_previous = 0.0f;
	msr_gyro_x_kalman_state.p_previous = GYRO_KALMAN_FILTER_P_INIT;

	// Y
	msr_gyro_y_kalman_params.f = GYRO_KALMAN_FILTER_F;
	msr_gyro_y_kalman_params.b = GYRO_KALMAN_FILTER_B;
	msr_gyro_y_kalman_params.h = GYRO_KALMAN_FILTER_H;
	msr_gyro_y_kalman_params.i = GYRO_KALMAN_FILTER_I;
	msr_gyro_y_kalman_params.q = GYRO_KALMAN_FILTER_Q;
	msr_gyro_y_kalman_params.r = GYRO_KALMAN_FILTER_R;

	msr_gyro_y_kalman_state.x_previous = 0.0f;
	msr_gyro_y_kalman_state.u_previous = 0.0f;
	msr_gyro_y_kalman_state.p_previous = GYRO_KALMAN_FILTER_P_INIT;

	// Z
	msr_gyro_z_kalman_params.f = GYRO_KALMAN_FILTER_F;
	msr_gyro_z_kalman_params.b = GYRO_KALMAN_FILTER_B;
	msr_gyro_z_kalman_params.h = GYRO_KALMAN_FILTER_H;
	msr_gyro_z_kalman_params.i = GYRO_KALMAN_FILTER_I;
	msr_gyro_z_kalman_params.q = GYRO_KALMAN_FILTER_Q;
	msr_gyro_z_kalman_params.r = GYRO_KALMAN_FILTER_R;

	msr_gyro_z_kalman_state.x_previous = 0.0f;
	msr_gyro_z_kalman_state.u_previous = 0.0f;
	msr_gyro_z_kalman_state.p_previous = GYRO_KALMAN_FILTER_P_INIT;
#else
	msr_gyro_x_filter2z_state.u_previous1 = 0.0f;
	msr_gyro_x_filter2z_state.y_previous1 = 0.0f;
	msr_gyro_x_filter2z_state.y_previous2 = 0.0f;

	msr_gyro_y_filter2z_state.u_previous1 = 0.0f;
	msr_gyro_y_filter2z_state.y_previous1 = 0.0f;
	msr_gyro_y_filter2z_state.y_previous2 = 0.0f;

	msr_gyro_z_filter2z_state.u_previous1 = 0.0f;
	msr_gyro_z_filter2z_state.y_previous1 = 0.0f;
	msr_gyro_z_filter2z_state.y_previous2 = 0.0f;
#endif

	// ACCEL

	msr_accel_filter_enable = true;

#if ACCEL_KALMAN_ENABLE
	// X
	msr_accel_x_kalman_params.f = ACCEL_KALMAN_FILTER_F;
	msr_accel_x_kalman_params.b = ACCEL_KALMAN_FILTER_B;
	msr_accel_x_kalman_params.h = ACCEL_KALMAN_FILTER_H;
	msr_accel_x_kalman_params.i = ACCEL_KALMAN_FILTER_I;
	msr_accel_x_kalman_params.q = ACCEL_KALMAN_FILTER_Q;
	msr_accel_x_kalman_params.r = ACCEL_KALMAN_FILTER_R;

	msr_accel_x_kalman_state.x_previous = 0.0f;
	msr_accel_x_kalman_state.u_previous = 0.0f;
	msr_accel_x_kalman_state.p_previous = ACCEL_KALMAN_FILTER_P_INIT;

	// Y
	msr_accel_y_kalman_params.f = ACCEL_KALMAN_FILTER_F;
	msr_accel_y_kalman_params.b = ACCEL_KALMAN_FILTER_B;
	msr_accel_y_kalman_params.h = ACCEL_KALMAN_FILTER_H;
	msr_accel_y_kalman_params.i = ACCEL_KALMAN_FILTER_I;
	msr_accel_y_kalman_params.q = ACCEL_KALMAN_FILTER_Q;
	msr_accel_y_kalman_params.r = ACCEL_KALMAN_FILTER_R;

	msr_accel_y_kalman_state.x_previous = 0.0f;
	msr_accel_y_kalman_state.u_previous = 0.0f;
	msr_accel_y_kalman_state.p_previous = ACCEL_KALMAN_FILTER_P_INIT;

	// Z
	msr_accel_z_kalman_params.f = ACCEL_KALMAN_FILTER_F;
	msr_accel_z_kalman_params.b = ACCEL_KALMAN_FILTER_B;
	msr_accel_z_kalman_params.h = ACCEL_KALMAN_FILTER_H;
	msr_accel_z_kalman_params.i = ACCEL_KALMAN_FILTER_I;
	msr_accel_z_kalman_params.q = ACCEL_KALMAN_FILTER_Q;
	msr_accel_z_kalman_params.r = ACCEL_KALMAN_FILTER_R;

	msr_accel_z_kalman_state.x_previous = 0.0f;
	msr_accel_z_kalman_state.u_previous = 0.0f;
	msr_accel_z_kalman_state.p_previous = ACCEL_KALMAN_FILTER_P_INIT;
#else
	msr_accel_x_filter2z_state.u_previous1 = 0.0f;
	msr_accel_x_filter2z_state.y_previous1 = 0.0f;
	msr_accel_x_filter2z_state.y_previous2 = 0.0f;

	msr_accel_y_filter2z_state.u_previous1 = 0.0f;
	msr_accel_y_filter2z_state.y_previous1 = 0.0f;
	msr_accel_y_filter2z_state.y_previous2 = 0.0f;

	msr_accel_z_filter2z_state.u_previous1 = 1000.0f;
	msr_accel_z_filter2z_state.y_previous1 = 1000.0f;
	msr_accel_z_filter2z_state.y_previous2 = 1000.0f;
#endif

	// MAG

	msr_mag_filter_enable = true;

	msr_mag_x_filter2z_state.u_previous1 = 0.0f;
	msr_mag_x_filter2z_state.y_previous1 = 0.0f;
	msr_mag_x_filter2z_state.y_previous2 = 0.0f;

	msr_mag_y_filter2z_state.u_previous1 = 0.0f;
	msr_mag_y_filter2z_state.y_previous1 = 0.0f;
	msr_mag_y_filter2z_state.y_previous2 = 0.0f;

	msr_mag_z_filter2z_state.u_previous1 = 0.0f;
	msr_mag_z_filter2z_state.y_previous1 = 0.0f;
	msr_mag_z_filter2z_state.y_previous2 = 0.0f;
}

#define MADGWICK_AHRS_BETA 0.25f

static void _init_madgwick_ahrs(void)
{
	madgwick_ahrs_init(&msr_madgwick_ahrs, MEASURE_TASK_PERIOD_MS * 0.001f, MADGWICK_AHRS_BETA);
}

static void _calibrate(struct accelerometer_driver *accel)
{
	static int step = 0;
	const int calib_steps = 2000 / MEASURE_TASK_PERIOD_MS;

	// Fc = 2Hz, Td=10 msec
	const struct filter2z_params calib_accel_filter2z_params = {
			0.014430,
			-1.82273, 0.83718
	};

	if (step > calib_steps)
	{
		step = 0;

		msr_accel_zero_level[0] = 0.0f - msr_calib_accel_x_filter2z_state.y;
		msr_accel_zero_level[1] = 0.0f - msr_calib_accel_y_filter2z_state.y;
		msr_accel_zero_level[2] = 1000.0f - msr_calib_accel_z_filter2z_state.y;

		hw_led_set_red(0);

		msr_state = MEASURE_NORMAL;
		return;
	}

	do
	{
		if (!accelerometer_open(accel))
			break;
		if (!accelerometer_read(accel, msr_raw_accel_data))
			break;

		if (step == 0)
		{
			msr_calib_accel_x_filter2z_state.u_previous1 = msr_raw_accel_data[0];
			msr_calib_accel_x_filter2z_state.y_previous1 = msr_raw_accel_data[0];
			msr_calib_accel_x_filter2z_state.y_previous2 = msr_raw_accel_data[0];

			msr_calib_accel_y_filter2z_state.u_previous1 = msr_raw_accel_data[1];
			msr_calib_accel_y_filter2z_state.y_previous1 = msr_raw_accel_data[1];
			msr_calib_accel_y_filter2z_state.y_previous2 = msr_raw_accel_data[1];

			msr_calib_accel_z_filter2z_state.u_previous1 = msr_raw_accel_data[2];
			msr_calib_accel_z_filter2z_state.y_previous1 = msr_raw_accel_data[2];
			msr_calib_accel_z_filter2z_state.y_previous2 = msr_raw_accel_data[2];

			hw_led_set_red(1);
		}
		else
		{
			filter2z_update(&calib_accel_filter2z_params, &msr_calib_accel_x_filter2z_state, msr_raw_accel_data[0]);
			filter2z_update(&calib_accel_filter2z_params, &msr_calib_accel_y_filter2z_state, msr_raw_accel_data[1]);
			filter2z_update(&calib_accel_filter2z_params, &msr_calib_accel_z_filter2z_state, msr_raw_accel_data[2]);
		}
	} while (0);

	(void)accelerometer_close(accel);

	++step;
}

static uint32_t msr_ticks = 0;

static void _measure(void const * argument)
{
	struct gyro_driver *gyro = 0;
	struct accelerometer_driver *accel = 0;
	struct magneto_driver *mag = 0;

	hw_mems_init();
	gyro = hw_get_gyro();
	accel = hw_get_accel();
	mag = hw_get_mag();

	_init_filters();
	_init_madgwick_ahrs();

	msr_ticks = osKernelSysTick();

	while (1)
	{
		uint32_t tmp;
		osDelayUntil(&msr_ticks, MEASURE_TASK_PERIOD_MS);

		tmp = osKernelSysTick();

		switch (msr_state)
		{
		case MEASURE_NORMAL:
			_get_gyro_data(gyro);
			_get_accel_data(accel);
			_get_mag_data(mag);
			_get_angle_data();
			break;
		case MEASURE_CALIBRATION:
			_calibrate(accel);
			break;
		}

		msr_measuring_duration = osKernelSysTick() - tmp + 1;
	}

	error: osDelay(1000); goto error;
}

void task_measure_init(void)
{
	osThreadDef(measure_task, _measure, osPriorityAboveNormal, 0, 1536);

	msr_mutex = osMutexCreate(NULL);
	task_measure = osThreadCreate(osThread(measure_task), NULL);
}

void get_gyro_data(float *data)
{
	osMutexWait(msr_mutex, osWaitForever);

	memcpy(data, msr_filtered_gyro_data, sizeof(msr_filtered_gyro_data));

	osMutexRelease(msr_mutex);
}

void get_accel_data(float *data)
{
	osMutexWait(msr_mutex, osWaitForever);

	memcpy(data, msr_filtered_accel_data, sizeof(msr_filtered_accel_data));

	osMutexRelease(msr_mutex);
}

void get_mag_data(float *data)
{
	osMutexWait(msr_mutex, osWaitForever);

	memcpy(data, msr_filtered_mag_data, sizeof(msr_filtered_mag_data));

	osMutexRelease(msr_mutex);
}

void get_angle_data(float *data)
{
	osMutexWait(msr_mutex, osWaitForever);

	memcpy(data, msr_angle_data, sizeof(msr_angle_data));

	osMutexRelease(msr_mutex);
}

uint32_t get_measuring_duration(void)
{
	return msr_measuring_duration;
}

void calibrate(void)
{
	msr_state = MEASURE_CALIBRATION;
}

bool calibration_in_progress(void)
{
	return (msr_state == MEASURE_CALIBRATION);
}
