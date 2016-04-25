#include "softpwm.h"

#define OFF (0)
#define ON (1)

extern unsigned long millis(void);

enum softpwm_phase
{
	IMPULSE_IDLE,
	IMPULSE_HIGH,
	IMPULSE_LOW
};

uint8_t softpwm_step(	const struct softpwm_params *params, 
						struct softpwm_state *state, 
						float x)
{
	uint8_t value = OFF;
	
	if (x > 1.0f)
		x = 1.0f;
	else if (x < 0.0f)
		x = 0.0f;
	
	if (x < params->duty_min)
	{
		state->phase = IMPULSE_IDLE;
		
		value = OFF;
		goto done;
	}
	
	if (x > params->duty_max)
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
		uint32_t impulse_duration = x * (params->period * 1000.0f);
	
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
		uint32_t pause_duration = (1.0f - x) * (params->period * 1000.0f);
		
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
