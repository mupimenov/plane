#ifndef CYCLOGRAM_H_
#define CYCLOGRAM_H_

#include <stdint.h>

struct cyclogram_params
{
	uint8_t type;
	
	uint16_t count;
	uint16_t impulse_duration;
	uint16_t pause_duration;
};

enum cyclogram_type
{
	CYCLOGRAM_SIMPLE,
	CYCLOGRAM_IMPULSE
};

struct cyclogram_state
{
	uint8_t phase;
	uint8_t remains;
	uint32_t start;
};

enum cyclogram_phase
{
	CYCLOGRAM_IDLE,
	
	CYCLOGRAM_HIGH,
	CYCLOGRAM_LOW
};

void cyclogram_start(	struct cyclogram_params *params,
						struct cyclogram_state *state);
						
uint8_t cyclogram_step(		struct cyclogram_params *params, 
							struct cyclogram_state *state);
							
bool cyclogram_is_stopped(struct cyclogram_state *state);

#endif /* CYCLOGRAM_H_ */
