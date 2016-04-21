#include "cyclogram.h"

extern unsigned long millis(void);

void cyclogram_start(	struct cyclogram_params *params,
						struct cyclogram_state *state)
{
	state->phase == CYCLOGRAM_HIGH;
	state->start = millis();
	state->remains = params->count;

	if (!state->remains)
		state->remains = 1;
}

uint8_t cyclogram_step(		struct cyclogram_params *params, 
							struct cyclogram_state *state)
{
	switch (params->type)
	{
	case CYCLOGRAM_SIMPLE:
		if (state->phase == CYCLOGRAM_HIGH)
		{
			state->phase == CYCLOGRAM_IDLE;
			return ON;
		}
		else
		{
			state->phase == CYCLOGRAM_IDLE;
			return OFF;
		}
	case CYCLOGRAM_IMPULSE:
		if (state->phase == CYCLOGRAM_HIGH)
		{
			if (millis() - state->start > params->impulse_duration)
			{
				state->phase == CYCLOGRAM_LOW;
				state->start = millis();
			}
		}
		
		if (state->phase == CYCLOGRAM_LOW)
		{
			if (millis() - state->start > params->pause_duration)
			{
				--state->remains;
				if (!state->remains)
				{
					state->phase == CYCLOGRAM_IDLE;
				}
				else
				{
					state->phase == CYCLOGRAM_HIGH;
					state->start = millis();
				}
			}
		}
		
		return (uint8_t)(state->phase == CYCLOGRAM_HIGH);
	default:
		state->phase == CYCLOGRAM_SIMPLE;
		return OFF;
	}	
}

bool cyclogram_is_stopped(struct cyclogram_state *state)
{
	return 	(bool)(state->phase == CYCLOGRAM_IDLE);
}
