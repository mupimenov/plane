#include "io.h"

#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "config.h"

#define ADC_CHANNELS_COUNT 16

#define DISCRETE_INPUT_FILTER_BITS 0x7

#define OFF (0)
#define ON (1)

extern unsigned long millis(void);

static uint16_t adc_value[ADC_CHANNELS_COUNT];

struct common_state
{
	uint8_t driver;
	uint8_t id;
};

struct analog_input_state
{
	uint8_t driver;
	uint8_t id;
	
	float value;
};

struct discrete_input_state
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t sequence;
	uint8_t value;
};

struct discrete_output_state
{
	uint8_t driver;
	uint8_t id;	
	
	uint8_t count;
	uint8_t value;
};

struct dht22_temperature_state
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t pin;	
	uint32_t last_millis;
	float value;
};

struct dht22_humidity_state
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t pin;	
	uint32_t last_millis;
	float value;
};

struct ioslot_state
{
	void (*execute)(struct ioslot_state *, struct abstract_ioslot *ioslot, uint8_t mode);
	void (*io_discrete)(struct ioslot_state *, uint8_t mode, uint8_t *value);
	void (*io_analog)(struct ioslot_state *, uint8_t mode, float *value);
	
	union {
		struct common_state 			common;
		struct analog_input_state 		analog_input;
		struct discrete_input_state 	discrete_input;
		struct discrete_output_state 	discrete_output;
		struct dht22_temperature_state 	dht22_temperature;
		struct dht22_humidity_state 	dht22_humidity;
	} data;
};

enum MODE
{
	IN,
	OUT
};

static struct ioslot_state 			ioslot_state[IOSLOTS_COUNT];

void io_lock(void)
{

}

void io_unlock(void)
{

}

void io_init(void)
{
	memset(adc_value, 0, sizeof(adc_value));
}

static void fill_adc_values(void)
{
	uint8_t i;
	
	for (i = 0; i < ADC_CHANNELS_COUNT; ++i)
	{
		adc_value[i] = i * 10 + 256;
	}
}

/* ANALOG INPUT */

static void analog_input_execute(struct ioslot_state *state, struct abstract_ioslot *ioslot, uint8_t mode)
{
	if (mode == IN)
	{
		state->data.analog_input.value = ioslot->data.analog_input.k
			* adc_value[ioslot->data.analog_input.num]
			+ ioslot->data.analog_input.b;
	}
}

static void analog_input_io_discrete(struct ioslot_state *state, uint8_t mode, uint8_t *value)
{
	if (mode == IN)
	{
		*value = 0;
	}
}

static void analog_input_io_analog(struct ioslot_state *state, uint8_t mode, float *value)
{
	if (mode == IN)
	{
		*value = state->data.analog_input.value;
	}
}

static bool prepare_analog_input(struct ioslot_state *state, struct abstract_ioslot *ioslot)
{
	if (ioslot->data.common.driver != ANALOG_INPUT_DRIVER)
		return false;
	
	state->execute = analog_input_execute;
	state->io_discrete = analog_input_io_discrete;
	state->io_analog = analog_input_io_analog;
	
	state->data.analog_input.driver = ioslot->data.analog_input.driver;
	state->data.analog_input.id = ioslot->data.analog_input.id;
	
	return true;
}

/* DISCRETE INPUT */

static void discrete_input_execute(struct ioslot_state *state, struct abstract_ioslot *ioslot, uint8_t mode)
{
	if (mode == IN)
	{
		state->data.discrete_input.sequence = (state->data.discrete_input.sequence << 1)
				| (state->data.discrete_input.id & 1);
			
		if ((state->data.discrete_input.value == OFF) 
			&& ((state->data.discrete_input.sequence & DISCRETE_INPUT_FILTER_BITS) == DISCRETE_INPUT_FILTER_BITS))
		{
			state->data.discrete_input.value = ON;
		}
		else if ((state->data.discrete_input.value == ON) 
			&& ((state->data.discrete_input.sequence & DISCRETE_INPUT_FILTER_BITS) == 0x0))
		{
			state->data.discrete_input.value = OFF;
		}
	}
}

static void discrete_input_io_discrete(struct ioslot_state *state, uint8_t mode, uint8_t *value)
{
	if (mode == IN)
	{
		*value = state->data.discrete_input.value;
	}
}

static void discrete_input_io_analog(struct ioslot_state *state, uint8_t mode, float *value)
{
	if (mode == IN)
	{
		*value = 1.0f * state->data.discrete_input.value;
	}
}

static bool prepare_discrete_input(struct ioslot_state *state, struct abstract_ioslot *ioslot)
{
	if (ioslot->data.common.driver != DISCRETE_INPUT_DRIVER)
		return false;
	
	state->execute = discrete_input_execute;
	state->io_discrete = discrete_input_io_discrete;
	state->io_analog = discrete_input_io_analog;
	
	state->data.discrete_input.driver = ioslot->data.discrete_input.driver;
	state->data.discrete_input.id = ioslot->data.discrete_input.id;
	
	return true;
}

/* DISCRETE OUTPUT */

static void discrete_output_execute(struct ioslot_state *state, struct abstract_ioslot *ioslot, uint8_t mode)
{
	if (mode == OUT)
	{
		if (ioslot->data.discrete_output.operation == OPERATION_OR)
		{
			state->data.discrete_output.value = state->data.discrete_output.count? ON: OFF;
		}
		else if (ioslot->data.discrete_output.operation == OPERATION_AND)
		{
			uint8_t max_count = program_count_with_output(state->data.discrete_output.id);
			state->data.discrete_output.value 
				= (state->data.discrete_output.count == max_count) ? ON: OFF;
		}
			
		state->data.discrete_output.count = 0;
	}
}

static void discrete_output_io_discrete(struct ioslot_state *state, uint8_t mode, uint8_t *value)
{
	if (mode == IN)
	{
		*value = state->data.discrete_output.value;
	}

	if (mode == OUT)
	{
		state->data.discrete_output.count++;
	}
}

static void discrete_output_io_analog(struct ioslot_state *state, uint8_t mode, float *value)
{
	if (mode == IN)
	{
		*value = (float)state->data.discrete_output.value;
	}

	if (mode == OUT)
	{
		if (*value >= 0.5f)
			state->data.discrete_output.count++;
	}
}

static bool prepare_discrete_output(struct ioslot_state *state, struct abstract_ioslot *ioslot)
{
	if (ioslot->data.common.driver != DISCRETE_OUTPUT_DRIVER)
		return false;
	
	state->execute = discrete_output_execute;
	state->io_discrete = discrete_output_io_discrete;
	state->io_analog = discrete_output_io_analog;
	
	state->data.discrete_output.driver = ioslot->data.discrete_output.driver;
	state->data.discrete_output.id = ioslot->data.discrete_output.id;
	
	return true;
}

/* DHT22 TEMPERATURE */

static void dht22_temperature_execute(struct ioslot_state *state, struct abstract_ioslot *ioslot, uint8_t mode)
{
	if (mode == IN)
	{
		
	}
}

static void dht22_temperature_io_discrete(struct ioslot_state *state, uint8_t mode, uint8_t *value)
{
	if (mode == IN)
	{
		*value = 0;
	}
}

static void dht22_temperature_io_analog(struct ioslot_state *state, uint8_t mode, float *value)
{
	if (mode == IN)
	{
		*value = state->data.dht22_temperature.value;
	}
}

static bool prepare_dht22_temperature(struct ioslot_state *state, struct abstract_ioslot *ioslot)
{
	if (ioslot->data.common.driver != DHT22_TEMPERATURE_DRIVER)
		return false;
	
	state->execute = dht22_temperature_execute;
	state->io_discrete = dht22_temperature_io_discrete;
	state->io_analog = dht22_temperature_io_analog;
	
	state->data.dht22_temperature.driver = ioslot->data.dht22_temperature.driver;
	state->data.dht22_temperature.id = ioslot->data.dht22_temperature.id;
	state->data.dht22_temperature.pin = ioslot->data.dht22_temperature.pin;
	
	return true;
}

/* DHT22 HUMIDITY */

static void dht22_humidity_execute(struct ioslot_state *state, struct abstract_ioslot *ioslot, uint8_t mode)
{
	if (mode == IN)
	{
		
	}
}

static void dht22_humidity_io_discrete(struct ioslot_state *state, uint8_t mode, uint8_t *value)
{
	if (mode == IN)
	{
		*value = 0;
	}
}

static void dht22_humidity_io_analog(struct ioslot_state *state, uint8_t mode, float *value)
{
	if (mode == IN)
	{
		*value = state->data.dht22_humidity.value;
	}
}

static bool prepare_dht22_humidity(struct ioslot_state *state, struct abstract_ioslot *ioslot)
{
	if (ioslot->data.common.driver != DHT22_HUMIDITY_DRIVER)
		return false;
	
	state->execute = dht22_humidity_execute;
	state->io_discrete = dht22_humidity_io_discrete;
	state->io_analog = dht22_humidity_io_analog;
	
	state->data.dht22_humidity.driver = ioslot->data.dht22_humidity.driver;
	state->data.dht22_humidity.id = ioslot->data.dht22_humidity.id;
	state->data.dht22_humidity.pin = ioslot->data.dht22_humidity.pin;
	
	return true;
}

static void update_dht22_state(uint8_t pin, uint32_t now, float temperature, float humidity)
{
	uint8_t i;
	
	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		if (ioslot_state[i].data.common.driver == DHT22_TEMPERATURE_DRIVER)
		{
			if (ioslot_state[i].data.dht22_temperature.pin == pin)
			{
				ioslot_state[i].data.dht22_temperature.last_millis = now;
				ioslot_state[i].data.dht22_temperature.value = temperature;
			}
		}
		
		if (ioslot_state[i].data.common.driver == DHT22_HUMIDITY_DRIVER)
		{
			if (ioslot_state[i].data.dht22_humidity.pin == pin)
			{
				ioslot_state[i].data.dht22_humidity.last_millis = now;
				ioslot_state[i].data.dht22_humidity.value = humidity;
			}
		}
	}
}

static void get_dht22_values(void)
{
	uint8_t i;
	bool dht22_requested = false;
	
	static uint8_t dht22_next_num = 0;
	static const uint32_t dht22_period = 2000000UL;
	
	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		if (ioslot_state[i].data.common.driver == DHT22_TEMPERATURE_DRIVER
			|| ioslot_state[i].data.common.driver == DHT22_HUMIDITY_DRIVER)
		{
			if (i >= dht22_next_num)
			{
				uint32_t now = millis();
				uint8_t pin = (ioslot_state[i].data.common.driver == DHT22_TEMPERATURE_DRIVER) ?
					ioslot_state[i].data.dht22_temperature.pin : 
					ioslot_state[i].data.dht22_humidity.pin;
				uint32_t last_millis = (ioslot_state[i].data.common.driver == DHT22_TEMPERATURE_DRIVER) ?	
					ioslot_state[i].data.dht22_temperature.last_millis : 
					ioslot_state[i].data.dht22_humidity.last_millis;
				
				if (last_millis + dht22_period > now
					|| now < last_millis)
				{
					float temperature = 10.0f + i * 0.5f;
					float humidity = 30.0f + i * 1.0f;
					
					update_dht22_state(pin, now, temperature, humidity);
					
					dht22_requested = true;
					dht22_next_num = i + 1;
					break;
				}
			}
		}
	}
	
	if (!dht22_requested)
	{
		dht22_next_num = 0;
	}
}

typedef bool (*prep_ioslot_fn)(struct ioslot_state *, struct abstract_ioslot *);

static const prep_ioslot_fn  prepare_ioslot[] = {
	prepare_analog_input,
	prepare_discrete_input,
	prepare_discrete_output,
	prepare_dht22_temperature,
	prepare_dht22_humidity
};

static const uint8_t prepare_ioslot_count = sizeof(prepare_ioslot) / sizeof(prepare_ioslot[0]);

static void io_prepare(void)
{
	uint8_t i;	
	
	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		struct abstract_ioslot ioslot;
		uint8_t j;
		
		read_ioslot(i, &ioslot);
		
		for (j = 0; j < prepare_ioslot_count; ++j)
		{
			if (prepare_ioslot[j](&ioslot_state[i], &ioslot))
				break;
		}
	}
}

void io_execute_in(void)
{	
	io_prepare();
	{
		uint8_t i;
	
		fill_adc_values();
	
		for (i = 0; i < IOSLOTS_COUNT; ++i)
		{
			struct abstract_ioslot ioslot;
			read_ioslot(i, &ioslot);
		
			if (ioslot_state[i].execute)
				ioslot_state[i].execute(&ioslot_state[i], &ioslot, IN);
		}
	
		get_dht22_values();
	}
}

void io_execute_out(void)
{
	uint8_t i;

	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		struct abstract_ioslot ioslot;
		read_ioslot(i, &ioslot);
		
		if (ioslot_state[i].execute)
			ioslot_state[i].execute(&ioslot_state[i], &ioslot, OUT);
	}
}

void ioslot_state_by_id(uint8_t id, struct ioslot_state **state)
{
	uint8_t i;

	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		if (ioslot_state[i].data.common.id == id)
		{
			*state = &ioslot_state[i];
			return;
		}
	}
	
	*state = 0;
}

uint8_t input_discrete(uint8_t id, int *err)
{
	struct ioslot_state *state;
	uint8_t value = 0;
	
	ioslot_state_by_id(id, &state);
	if (!state)
	{
		*err = 1;
		return value;
	}
	
	if (state->io_discrete)
		state->io_discrete(state, IN, &value);
	return value;
}

float input_analog(uint8_t id, int *err)
{
	struct ioslot_state *state;
	float value = NAN;
	
	ioslot_state_by_id(id, &state);
	if (!state)
	{
		*err = 1;
		return value;
	}
	
	if (state->io_analog)
		state->io_analog(state, IN, &value);
	return value;
}

void output_discrete(uint8_t id, uint8_t value, int *err)
{
	struct ioslot_state *state;
	
	ioslot_state_by_id(id, &state);
	if (!state)
	{
		*err = 1;
		return;
	}
	
	if (state->io_discrete)
		state->io_discrete(state, OUT, &value);
}

uint16_t get_adc_value(uint8_t channel)
{
	return adc_value[channel];
}

float get_ioslot_value(uint8_t num)
{
	float value = NAN;
	struct ioslot_state *state = &ioslot_state[num];

	if (state->io_analog)
		state->io_analog(state, IN, &value);

	return value;
}
