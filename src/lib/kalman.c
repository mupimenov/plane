/*
 * kalman.c
 *
 *  Created on: Feb 27, 2016
 *      Author: mupimenov
 */

#include "kalman.h"

#include <string.h>

static void _matrix_3x1_add_3x1(float m1[3][1], /*const*/ float m2[3][1])
{
	int i;
	for (i = 0; i < 3; ++i)
		m1[i][0] = m1[i][0] + m2[i][0];
}

static void _matrix_3x3_add_3x3(float m1[3][3], /*const*/ float m2[3][3])
{
	int i;
	for (i = 0; i < 3; ++i)
	{
		int j;
		for (j = 0; j < 3; ++j)
			m1[i][j] = m1[i][j] + m2[i][j];
	}
}

static void _matrix_3x1_mul_constant(float m[3][1], float a)
{
	int i;
	for (i = 0; i < 3; ++i)
	{
		m[i][0] = a * m[i][0];
	}
}

static void _matrix_3x3_mul_constant(float m[3][3], float a)
{
	int i;
	for (i = 0; i < 3; ++i)
	{
		int j;
		for (j = 0; j < 3; ++j)
		{
			m[i][j] = a * m[i][j];
		}
	}
}

static void _matrix_3x3_mul_3x1(/*const*/ float m1[3][3], /*const*/ float m2[3][1], float m3[3][1])
{
	m3[0][0] = m1[0][0] * m2[0][0] +  m1[0][1] * m2[1][0] +  m1[0][2] * m2[2][0];
	m3[1][0] = m1[1][0] * m2[0][0] +  m1[1][1] * m2[1][0] +  m1[1][2] * m2[2][0];
	m3[2][0] = m1[2][0] * m2[0][0] +  m1[2][1] * m2[1][0] +  m1[2][2] * m2[2][0];
}

static void _matrix_3x3_mul_3x3(/*const*/ float m1[3][3], /*const*/ float m2[3][3], float m3[3][3])
{
	int i;
	for (i = 0; i < 3; ++i)
	{
		m3[0][i] = m1[0][0] * m2[0][i] +  m1[0][1] * m2[1][i] +  m1[0][2] * m2[2][i];
		m3[1][i] = m1[1][0] * m2[0][i] +  m1[1][1] * m2[1][i] +  m1[1][2] * m2[2][i];
		m3[2][i] = m1[2][0] * m2[0][i] +  m1[2][1] * m2[1][i] +  m1[2][2] * m2[2][i];
	}
}

static void _matrix_3x3_transpose(/*const*/ float m1[3][3], float m2[3][3])
{
	int i;
	for (i = 0; i < 3; ++i)
	{
		int j;
		for (j = 0; j < 3; ++j)
		{
			if (i == j)
				continue;
			m2[j][i] = m1[i][j];
		}
	}
}

static float _matrix_2x2_determinant(/*const*/ float m[2][2])
{
	return (m[0][0] * m[1][1] - m[0][1] * m[1][0]);
}

static float _matrix_3x3_determinant(/*const*/ float m[3][3])
{
	float tmp[2][2];
	float result;

	tmp[0][0] = m[1][1];
	tmp[0][1] = m[1][2];
	tmp[1][0] = m[2][1];
	tmp[1][1] = m[2][2];

	result = m[0][0] * _matrix_2x2_determinant(tmp);

	tmp[0][0] = m[1][0];
	tmp[0][1] = m[1][2];
	tmp[1][0] = m[2][0];
	tmp[1][1] = m[2][2];

	result -= m[0][1] * _matrix_2x2_determinant(tmp);

	tmp[0][0] = m[1][0];
	tmp[0][1] = m[1][1];
	tmp[1][0] = m[2][0];
	tmp[1][1] = m[2][1];

	result += m[0][1] * _matrix_2x2_determinant(tmp);

	return result;
}

static void _matrix_3x3_invert(/*const*/ float m1[3][3], float m2[3][3])
{
	float tmp[2][2];
	float determinant = _matrix_3x3_determinant(m1);
	if (determinant == 0.0f)
		return;

	tmp[0][0] = m1[1][1];
	tmp[0][1] = m1[1][2];
	tmp[1][0] = m1[2][1];
	tmp[1][1] = m1[2][2];

	m2[0][0] = _matrix_2x2_determinant(tmp);

	tmp[0][0] = m1[1][0];
	tmp[0][1] = m1[1][2];
	tmp[1][0] = m1[2][0];
	tmp[1][1] = m1[2][2];

	m2[0][1] = _matrix_2x2_determinant(tmp);

	tmp[0][0] = m1[1][0];
	tmp[0][1] = m1[1][1];
	tmp[1][0] = m1[2][0];
	tmp[1][1] = m1[2][1];

	m2[0][2] = _matrix_2x2_determinant(tmp);

	tmp[0][0] = m1[0][1];
	tmp[0][1] = m1[0][2];
	tmp[1][0] = m1[2][1];
	tmp[1][1] = m1[2][2];

	m2[1][0] = _matrix_2x2_determinant(tmp);

	tmp[0][0] = m1[0][0];
	tmp[0][1] = m1[0][2];
	tmp[1][0] = m1[2][0];
	tmp[1][1] = m1[2][2];

	m2[1][1] = _matrix_2x2_determinant(tmp);

	tmp[0][0] = m1[0][0];
	tmp[0][1] = m1[0][1];
	tmp[1][0] = m1[2][0];
	tmp[1][1] = m1[2][1];

	m2[1][2] = _matrix_2x2_determinant(tmp);

	tmp[0][0] = m1[0][1];
	tmp[0][1] = m1[0][2];
	tmp[1][0] = m1[1][1];
	tmp[1][1] = m1[1][2];

	m2[2][0] = _matrix_2x2_determinant(tmp);

	tmp[0][0] = m1[0][0];
	tmp[0][1] = m1[0][2];
	tmp[1][0] = m1[1][0];
	tmp[1][1] = m1[1][2];

	m2[2][1] = _matrix_2x2_determinant(tmp);

	tmp[0][0] = m1[0][0];
	tmp[0][1] = m1[0][1];
	tmp[1][0] = m1[1][0];
	tmp[1][1] = m1[1][1];

	m2[2][2] = _matrix_2x2_determinant(tmp);

	_matrix_3x3_mul_constant(m2, 1.0f / determinant);
}

void kalman3_correct(/*const*/ struct kalman3_params *params, struct kalman3_state *state, /*const*/ float z[3], /*const*/ float u[3])
{
	float x_predicted[3][1];
	float p_predicted[3][3];
	float kalman_gain[3][3];

	float tmp_3x1[3][1];
	float tmp2_3x1[3][1];
	float tmp_3x3[3][3];
	float tmp2_3x3[3][3];
	float extra_3x3[3][3];

	// x predicted
	_matrix_3x3_mul_3x1(params->f, state->x_previous, x_predicted);
	_matrix_3x3_mul_3x1(params->b, state->u_previous, tmp_3x1);

	_matrix_3x1_add_3x1(x_predicted, tmp_3x1);

	// p predicted
	_matrix_3x3_mul_3x3(params->f, state->p_previous, tmp_3x3);
	_matrix_3x3_transpose(params->f, tmp2_3x3);

	_matrix_3x3_mul_3x3(tmp_3x3, tmp2_3x3, p_predicted);

	_matrix_3x3_add_3x3(p_predicted, params->q);

	// kalman gain
	_matrix_3x3_mul_3x3(params->h, p_predicted, tmp_3x3);
	_matrix_3x3_transpose(params->h, tmp2_3x3);

	_matrix_3x3_mul_3x3(tmp_3x3, tmp2_3x3, extra_3x3);
	_matrix_3x3_add_3x3(extra_3x3, params->r);

	_matrix_3x3_invert(extra_3x3, tmp_3x3);

	_matrix_3x3_mul_3x3(p_predicted, tmp2_3x3, extra_3x3);

	_matrix_3x3_mul_3x3(extra_3x3, tmp_3x3, kalman_gain);

	// x state
	_matrix_3x3_mul_3x1(params->h, x_predicted, tmp_3x1);
	_matrix_3x1_mul_constant(tmp_3x1, -1.0f);

	memcpy(tmp2_3x1, z, sizeof(tmp2_3x1));
	_matrix_3x1_add_3x1(tmp2_3x1, tmp_3x1);

	_matrix_3x3_mul_3x1(kalman_gain, tmp2_3x1, state->x_previous);
	_matrix_3x1_add_3x1(state->x_previous, x_predicted);

	// p state
	_matrix_3x3_mul_3x3(kalman_gain, params->h, tmp_3x3);
	_matrix_3x3_mul_constant(tmp_3x3, -1.0f);

	_matrix_3x3_add_3x3(tmp_3x3, params->i);

	_matrix_3x3_mul_3x3(tmp_3x3, p_predicted, state->p_previous);

	// u state
	memcpy(state->u_previous, u, sizeof(state->u_previous));
}

void kalman1_correct(/*const*/ struct kalman1_params *params, struct kalman1_state *state, /*const*/ float z, /*const*/ float u)
{
	float x_predicted;
	float p_predicted;
	float kalman_gain;

	// x predicted
	x_predicted = params->f * state->x_previous + params->b * state->u_previous;

	// p predicted
	p_predicted = params->f * state->p_previous * params->f + params->q;

	// kalman gain
	kalman_gain = p_predicted * params->h / (params->h * p_predicted * params->h + params->r);

	// x state
	state->x_previous = kalman_gain * (z - params->h * x_predicted) + x_predicted;

	// p state
	state->p_previous = (params->i - kalman_gain * params->h) * p_predicted;

	// u state
	state->u_previous = u;
}
