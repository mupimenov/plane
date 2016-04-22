#include "program.h"

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
	struct softpwm_state softpwm;
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
	memset(states, 0, sizeof(states));
}

static void timer_control_execute(struct timer_program *program, struct timer_state *state);
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
			timer_control_execute(&program.data.timer, &states[num].timer);
			break;
		case RELAY_CONTROL_PROGRAM:
			relay_control_execute(&program.data.relay, &states[num].relay);
			break;
		case PID_CONTROL_PROGRAM:
			pid_control_execute(&program.data.pid, &states[num].pid);
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
			
			if (!datetime_in(&now, program->from, program->to, program->constrains))
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
			
			if (state->value == ON)
				state->phase = RELAY_OUTPUT;
			
		} while (0);
	}
	
	if (state->phase == RELAY_OUTPUT)
	{
		if (cyclogram_is_stopped(state->cyclogram))
			cyclogram_start(program->cyclogram, state->cyclogram);
		
		value = cyclogram_step(program->cyclogram, state->cyclogram);
		
		if (cyclogram_is_stopped(state->cyclogram))
			state->phase = RELAY_COMPUTE;
	}
	
	output_discrete(program->output, value, &err);
}

/* PID CONTROL PROGRAM */

static void pid_control_execute(struct pid_control_program *program, struct pid_control_state *state)
{
	const uint32_t step = 10;
	const struct softpwm_params softpwm = {
		5, 0.2f, 0.8f
	};
	
	int err;	
	uint32_t delta_millis;	
	
	delta_millis = millis() - state->last_millis;
	if (current_millis > step)
	{
		do
		{
			float dt = (float)delta_millis / 1000.0f;
			
			float proportional;
			float differential;			
			
			float dx;
			float y;
			
			float x = input_analog(program->input, &err);
			if (err)
				break;
			
			dx = program->need - x;
			
			proportional = program->proportional_gain * dx;
			if (proportional > 1.0f)
				proportional = 1.0f;
			else if (proportional < -1.0f)
				proportional = -1.0f;
			
			if (state->integral_sum == NAN)
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
			
			state->value = softpwm_step(&softpwm, state->softpwm, y);
			
		} while (0);
	}
	
	output_discrete(program->output, state->value, &err);
}
