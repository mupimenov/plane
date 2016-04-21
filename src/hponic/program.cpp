#include "program.h"

#define OFF (0)
#define ON (1)

struct timer_state
{
	uint8_t phase;
	struct cyclogram_state cyclogram;
};

enum timer_phase
{
	TIMER_COMPUTE,
	TIMER_OUTPUT
};

union abstract_state
{
	struct timer_state timer;
};

static union abstract_state states[PROGRAMS_COUNT];

void program_init(void)
{
	memset(states, 0, sizeof(states));
}

static void timer_execute(struct timer_program *program, struct timer_state *state);

void program_execute(void)
{
	int num;
	for (num = 0; num < PROGRAMS_COUNT; ++num)
	{
		union abstract_program program;
		read_program(num, &program);
		
		switch (program.common.type)
		{
		case PROGRAM_TIMER:
			timer_execute(&program.timer, &states[num]);
		default:
			break;
		}
	}
}

/* TIMER PROGRAM */

static void timer_execute(struct timer_program *program, struct timer_state *state)
{
	uint8_t value = OFF;
	
	if (state->phase != TIMER_COMPUTE
		&& state->phase != TIMER_OUTPUT)
	{
		state->phase = TIMER_COMPUTE;
	}

	if (state->phase == TIMER_COMPUTE)
	{
		struct datetime now = datetime_now();
		if (datetime_in(&now, program->from, program->to, program->constrains))
		{
			state->phase = TIMER_OUTPUT;
		}
	}
	
	if (state->phase == TIMER_OUTPUT)
	{
		if (cyclogram_is_stopped(state->cyclogram))
			cyclogram_start(program->cyclogram, state->cyclogram);
		
		value = cyclogram_step(program->cyclogram, state->cyclogram);
		
		if (cyclogram_is_stopped(state->cyclogram))
			state->phase = TIMER_COMPUTE;
	}
	
	output_discrete(program->discrete, value);
}
