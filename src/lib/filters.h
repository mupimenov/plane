/*
 * filters.h
 *
 *  Created on: Mar 6, 2016
 *      Author: mupimenov
 */

#ifndef SRC_LIB_FILTERS_H_
#define SRC_LIB_FILTERS_H_

struct filter2z_params {
	float b1;
	float a1;
	float a2;
};

struct filter2z_state {
	float u_previous1;
	float y;
	float y_previous1;
	float y_previous2;
};

void filter2z_update(const struct filter2z_params *params, struct filter2z_state *state, float u);

#endif /* SRC_LIB_FILTERS_H_ */
