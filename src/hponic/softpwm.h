#ifndef SOFTPWM_H_
#define SOFTPWM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct softpwm_params
{
	uint8_t period;
	uint8_t duration_min;
};

struct softpwm_state
{
	uint8_t phase;
	uint32_t start;
	float value;
};

uint8_t softpwm_step(	const struct softpwm_params *params,
						struct softpwm_state *state,
						float x);

#ifdef __cplusplus
}
#endif

#endif /* SOFTPWM_H_ */
