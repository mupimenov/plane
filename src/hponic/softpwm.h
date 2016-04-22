#ifndef SOFTPWM_H_
#define SOFTPWM_H_

#include <stdint.h>

struct softpwm_params
{
	uint8_t period;
	float duty_min;
	float duty_max;
};

struct softpwm_state
{
	uint8_t phase;
	uint32_t start;
	float value;
};

#endif /* SOFTPWM_H_ */
