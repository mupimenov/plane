#include "softpwm.h"

#define OFF (0)
#define ON (1)

extern unsigned long millis(void);

void softpwm_reset(struct softpwm_state *state)
{
	state->phase = IMPULSE_IDLE;
	state->start = 0;
	state->value = 0.0f;
}

uint8_t softpwm_step(	struct softpwm_state *state,
						const struct softpwm_params *params
						float x)
{
	uint8_t value = OFF;
	uint32_t impulse_duration;
	uint32_t pause_duration;
	
	if (x > 1.0f)
		x = 1.0f;
	else if (x < 0.0f)
		x = 0.0f;
	
	impulse_duration = x * (params->period * 1000.0f);
	if (impulse_duration < params->duration_min)
	{
		state->phase = IMPULSE_IDLE;
		
		value = OFF;
		goto done;
	}
	
	pause_duration = (1.0f - x) * (params->period * 1000.0f);
	if (pause_duration < params->duration_min)
	{
		state->phase = IMPULSE_IDLE;
		
		value = ON;
		goto done;
	}
	
	if (state->phase == IMPULSE_IDLE)
	{
		state->start = millis();
		state->phase = IMPULSE_HIGH;
	}
	
	if (state->phase == IMPULSE_HIGH)
	{
		if (state->start + impulse_duration > millis())
		{
			state->start = millis();
			state->phase = IMPULSE_LOW;
			
			value = OFF;
			goto done;
		}
		else
		{
			value = ON;
			goto done;
		}
	}
	
	if (state->phase == IMPULSE_LOW)
	{
		if (state->start + pause_duration > millis())
		{
			state->start = millis();
			state->phase = IMPULSE_HIGH;
			
			value = ON;
			goto done;
		}
		else
		{
			value = OFF;
			goto done;
		}
	}
	
done:
	return value;
}
