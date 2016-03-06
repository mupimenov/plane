/*
 * kalman.h
 *
 *  Created on: Feb 27, 2016
 *      Author: mupimenov
 */

#ifndef SRC_LIB_KALMAN_H_
#define SRC_LIB_KALMAN_H_

struct kalman3_params {
	float f[3][3];
	float b[3][3]; // ?
	float q[3][3];
	float h[3][3];
	float r[3][3];
	float i[3][3];
};

struct kalman3_state {
	float x_previous[3][1];
	float u_previous[3][1]; // like a `b`
	float p_previous[3][3];
};

void kalman3_correct(/*const*/ struct kalman3_params *params, struct kalman3_state *state, /*const*/ float z[3], /*const*/ float u[3]);

struct kalman1_params {
	float f;
	float b; // ?
	float q;
	float h;
	float r;
	float i;
};

struct kalman1_state {
	float x_previous;
	float u_previous; // like a `b`
	float p_previous;
};

void kalman1_correct(/*const*/ struct kalman1_params *params, struct kalman1_state *state, /*const*/ float z, /*const*/ float u);

#endif /* SRC_LIB_KALMAN_H_ */
