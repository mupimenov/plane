#include "program.h"

#include <string.h>
#include <math.h>

#include "config.h"
#include "datetime.h"
#include "cyclogram.h"
#include "softpwm.h"

#define OFF (0)
#define ON (1)

extern unsigned long millis(void);

/* TIMER CONTROL */

struct timer_control_state
{
	uint8_t phase;
	struct cyclogram_state cyclogram;
};

enum timer_control_phase
{
	TIMER_COMPUTE,
	TIMER_OUTPUT
};

/* RELAY CONTROL */

struct relay_control_state
{
	uint8_t phase;
	uint8_t value;
	struct cyclogram_state cyclogram;
};

enum relay_control_phase
{
	RELAY_COMPUTE,
	RELAY_OUTPUT
};

/* PID CONTROL */

struct pid_control_state
{
	float integral_sum;	
	uint32_t last_millis;
	struct softpwm_params softpwm_p;
	struct softpwm_state softpwm_s;
	uint8_t value;
};

union abstract_program_state
{
	struct timer_control_state timer;
	struct relay_control_state relay;
	struct pid_control_state pid;
};

static union abstract_program_state program_state[PROGRAMS_COUNT];

void program_init(void)
{
	memset(program_state, 0, sizeof(program_state));
}

static void timer_control_execute(struct timer_control_program *program, struct timer_control_state *state);
static void relay_control_execute(struct relay_control_program *program, struct relay_control_state *state);
static void pid_control_execute(struct pid_control_program *program, struct pid_control_state *state);

void program_execute(void)
{
	int num;

	for (num = 0; num < PROGRAMS_COUNT; ++num)
	{
		struct abstract_program program;
		read_program(num, &program);
		
		switch (program.data.common.type)
		{
		case TIMER_CONTROL_PROGRAM:
			timer_control_execute(&program.data.timer, &program_state[num].timer);
			break;
		case RELAY_CONTROL_PROGRAM:
			relay_control_execute(&program.data.relay, &program_state[num].relay);
			break;
		case PID_CONTROL_PROGRAM:
			pid_control_execute(&program.data.pid, &program_state[num].pid);
			break;
		default:
			break;
		}
	}
}

/* TIMER CONTROL PROGRAM */

static void timer_control_execute(struct timer_control_program *program, struct timer_control_state *state)
{
	uint8_t value = OFF;
	int err;
	
	if (state->phase != TIMER_COMPUTE
		&& state->phase != TIMER_OUTPUT)
	{
		state->phase = TIMER_COMPUTE;
	}

	if (state->phase == TIMER_COMPUTE)
	{
		struct datetime now = datetime_now();
		if (datetime_in(&now, &program->from, &program->to, (enum time_constrains)program->constrains))
		{
			state->phase = TIMER_OUTPUT;
		}
	}
	
	if (state->phase == TIMER_OUTPUT)
	{
		if (cyclogram_is_stopped(&state->cyclogram))
			cyclogram_start(&program->cyclogram, &state->cyclogram);
		
		value = cyclogram_step(&program->cyclogram, &state->cyclogram);
		
		if (cyclogram_is_stopped(&state->cyclogram))
			state->phase = TIMER_COMPUTE;
	}
	
	output_discrete(program->output, value, &err);
}

/* RELAY CONTROL PROGRAM */

static void relay_control_execute(struct relay_control_program *program, struct relay_control_state *state)
{
	uint8_t value = OFF;
	int err;
	
	if (state->phase != RELAY_COMPUTE
		&& state->phase != RELAY_OUTPUT)
	{
		state->phase = RELAY_COMPUTE;
	}

	if (state->phase == RELAY_COMPUTE)
	{
		do
		{
			float input;
			int err;
			struct datetime now = datetime_now();
			
			if (!datetime_in(&now, &program->from, &program->to, (enum time_constrains)program->constrains))
				break;
			
			input = input_analog(program->input, &err);
			if (err)
				break;
			
			if (state->value == OFF && input < program->low_bound)
			{
				state->value = ON;
			}
			else if (state->value == ON && input < program->high_bound)
			{
				state->value = ON;
			}
			else if (state->value == ON && input >= program->high_bound)
			{
				state->value = OFF;
			}
			else if (state->value == OFF && input >= program->low_bound)
			{
				state->value = OFF;
			}
			
			if (!program->inverse)
			{
				if (state->value == ON)
					state->phase = RELAY_OUTPUT;
			}
			else
			{
				if (state->value == OFF)
					state->phase = RELAY_OUTPUT;
			}
			
		} while (0);
	}
	
	if (state->phase == RELAY_OUTPUT)
	{
		if (cyclogram_is_stopped(&state->cyclogram))
			cyclogram_start(&program->cyclogram, &state->cyclogram);
		
		value = cyclogram_step(&program->cyclogram, &state->cyclogram);
		
		if (cyclogram_is_stopped(&state->cyclogram))
			state->phase = RELAY_COMPUTE;
	}
	
	output_discrete(program->output, value, &err);
}

/* PID CONTROL PROGRAM */

static void pid_control_execute(struct pid_control_program *program, struct pid_control_state *state)
{
	const uint32_t step = 10;
	const float period_min = 5.0f;
	const float period_max = 60.0f;
	const float period_gain = 30.0f;
	const uint8_t duration_min = 1; /* seconds */
	const float dx_min = 0.1f;
	
	int err;
	uint32_t delta_millis;
	
	delta_millis = millis() - state->last_millis;
	if (delta_millis > step)
	{
		do
		{
			float dt = (float)delta_millis / 1000.0f;
			
			float proportional;
			float differential;			
			
			float dx;
			float period;
			float y;
			
			float x = input_analog(program->input, &err);
			if (err)
				break;
			
			dx = program->desired - x;
			
			proportional = program->proportional_gain * dx;
			if (proportional > 1.0f)
				proportional = 1.0f;
			else if (proportional < -1.0f)
				proportional = -1.0f;
			
			if (isnan(state->integral_sum))
				state->integral_sum = 0.0f;
			state->integral_sum += program->integral_gain * dx * dt;
			
			if (state->integral_sum > 1.0f)
				state->integral_sum = 1.0f;
			else if (state->integral_sum < -1.0f)
				state->integral_sum = -1.0f;
			
			differential = dx / dt;
			if (differential > 1.0f)
				differential = 1.0f;
			else if (differential < -1.0f)
				differential = -1.0f;
			
			y = proportional + state->integral_sum + differential;
			if (program->inverse)
			{
				y *= -1.0f;
			}
			
			if (fabs(dx) < dx_min)
			{
				period = period_max;
			}
			else
			{
				period = period_gain / fabs(dx);
				
				if (period < period_min)
					period = period_min;
				else if (period > period_max)
					period = period_max;
			}
			
			state->softpwm_p.period = period;
			state->softpwm_p.duration_min = duration_min;
			
			state->value = softpwm_step(&state->softpwm_p, &state->softpwm_s, y);
			
		} while (0);
	}
	
	output_discrete(program->output, state->value, &err);
}
