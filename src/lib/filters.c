/*
 * filters.c
 *
 *  Created on: Mar 6, 2016
 *      Author: mupimenov
 */

#include "filters.h"

void filter2z_update(const struct filter2z_params *params, struct filter2z_state *state, float u)
{
	state->y = (params->b1 * state->u_previous1) -
			(params->a1 * state->y_previous1 + params->a2 * state->y_previous2);

	state->y_previous2 = state->y_previous1;
	state->y_previous1 = state->y;
	state->u_previous1 = u;
}
