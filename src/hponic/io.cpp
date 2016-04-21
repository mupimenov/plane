#include "io.h"

#include "config.h"

#define ADC_CHANNELS_COUNT 16

static uint16_t adc_value[ADC_CHANNELS_COUNT];

struct analog_input_state
{
	uint8_t driver;
	float value;
};

struct discrete_input_state
{
	uint8_t driver;
	uint8_t sequence;
	uint8_t value;
};

struct discrete_output_state
{
	uint8_t driver;
	uint8_t count;
	uint8_t value;
};

struct dht22_temperature_state
{
	uint8_t driver;
	quint32_t last_millis;
	float value;
};

struct dht22_humidity_state
{
	uint8_t driver;
	quint32_t last_millis;
	float value;
};

union ioslot_state
{
	struct analog_input_state 		analog_input;
	struct discrete_input_state 	discrete_input;
	struct discrete_output_state 	discrete_output;
	struct dht22_temperature_state 	dht22_temperature;
	struct dht22_humidity_state 	dht22_humidity;
};

static union ioslot_state 			ioslot_state[IOSLOTS_COUNT];

void io_init(void)
{
	memset(adc_value, 0, sizeof(adc_value));
}

static void get_adc_values(void)
{
	uint8_t i;
	
	for (i = 0; i < ADC_CHANNELS_COUNT; ++i)
	{
		adc_value[i] = i * 10 + 256;
	}
}

void io_execute_in(void)
{	
	uint16_t i;
	
	get_adc_values();
	
	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		union abstract_io_slot io_slot;
		read_io_slot(i, &io_slot);
		
		switch (io_slot.common.driver)
		{
		case EMPTY_SLOT_DRIVER:
			break;
		case ANALOG_INPUT_DRIVER:
			ioslot_state[i].analog_input.driver = io_slot.analog_input.driver;
			ioslot_state[i].analog_input.value = io_slot.analog_input.k * adc_value[io_slot.analog_input.num]
				+ io_slot.analog_input.b;
			break;
		case DISCRETE_INPUT_DRIVER:
			break;
		case DISCRETE_OUTPUT_DRIVER:
			break;
		case DHT22_TEMPERATURE_DRIVER:
			break;
		case DHT22_HUMIDITY_DRIVER:
			break;
		default:
			break;
		}
	}
}

void io_execute_out(void)
{
	
}

uint8_t input_discrete(uint8_t id, int *err)
{
	
}

float input_analog(uint8_t id, int *err)
{
	
}

void output_discrete(uint8_t id, uint8_t value)
{
	
}
