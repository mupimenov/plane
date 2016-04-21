#include "config.h"

#define IO_SLOT_START_ADDRESS 0x0000
#define IO_SLOT_SIZE 16

#define PROGRAM_START_ADDRESS 0x0400
#define PROGRAM_SIZE 48

static uint8_t cfg_file[4096];

void config_init(void)
{
	memset(cfg_file, 0, sizeof(cfg_file));
}

void config_lock(void)
{
	
}

void config_unlock(void)
{
	
}

void config_read(uint16_d address, uint8_t *data, uint16_t size)
{
	uint16_t i;
	for (i = 0; i < size; ++i)
	{
		data[i] = cfg_file[address + i];
	}
}

void config_write(uint16_d address, const uint8_t *data, uint16_t size)
{
	uint16_t i;
	for (i = 0; i < size; ++i)
	{
		cfg_file[address + i] = data[i];
	}
}

static float array_to_float(const uint8_t *arr, int offset)
{
	uint32_t v = (uint32_t)arr[offset]
		| ((uint32_t)arr[offset + 1] << 8)
		| ((uint32_t)arr[offset + 2] << 16)
		| ((uint32_t)arr[offset + 3] << 24);
	return *((float*)&v);
}

void read_io_slot(uint8_t num, union abstract_io_slot *io_slot)
{
	uint16_t address = IO_SLOT_START_ADDRESS + num * IO_SLOT_SIZE;
	
	io_slot->common.driver = cfg_file[address];
	io_slot->common.id = cfg_file[address + 1];
	
	switch (io_slot->common.driver)
	{
	case EMPTY_SLOT_DRIVER:
		break;
	case ANALOG_INPUT_DRIVER:
		io_slot->analog_input.num = cfg_file[address + 2];
		io_slot->analog_input.k = array_to_float(&cfg_file[address], 3);
		io_slot->analog_input.b = array_to_float(&cfg_file[address], 7);
		break;
	case DISCRETE_INPUT_DRIVER:
		io_slot->discrete_input.pin = cfg_file[address + 2];
		io_slot->discrete_input.inverse = (cfg_file[address + 3] == 0x01);
		break;
	case DISCRETE_OUTPUT_DRIVER:
		io_slot->discrete_output.operation = cfg_file[address + 2];
		io_slot->discrete_output.pin = cfg_file[address + 3];
		io_slot->discrete_output.inverse = (cfg_file[address + 4] == 0x01);
		break;
	case DHT22_TEMPERATURE_DRIVER:
		io_slot->dht22_temperature.pin = cfg_file[address + 2];
		break;
	case DHT22_HUMIDITY_DRIVER:
		io_slot->dht22_humidity.pin = cfg_file[address + 2];
		break;
	default:
		io_slot->common.driver = EMPTY_SLOT_DRIVER;
		io_slot->common.id = 0;
		break;
	}
}

static void array_to_datetime(const uint8_t *arr, int offset, struct datetime *dt)
{
	dt->seconds = arr[offset];
	dt->minutes = arr[offset + 1];
	dt->hours = arr[offset + 2];
	dt->day = arr[offset + 3];
	dt->month = arr[offset + 4];
	dt->year = arr[offset + 5];
}

static void array_to_cyclogram(const uint8_t *arr, int offset, struct cyclogram_params *cyclogram)
{
	cyclogram->type = arr[offset];
	cyclogram->count = (uint16_t)arr[offset + 1] | ((uint16_t)arr[offset + 2] << 8);
	cyclogram->impulse_duration = (uint16_t)arr[offset + 3] | ((uint16_t)arr[offset + 4] << 8);
	cyclogram->pause_duration = (uint16_t)arr[offset + 5] | ((uint16_t)arr[offset + 6] << 8);
}

void read_program(uint8_t num, union abstract_program *program)
{
	uint16_t address = PROGRAM_START_ADDRESS + num * PROGRAM_SIZE;
	
	program->common.type = cfg_file[address];
	program->common.id = cfg_file[address + 1];
	
	switch (program->common.type)
	{
	case EMPTY_PROGRAM:
		break;
	case TIMER_CONTROL_PROGRAM:
		program->timer.constrains = cfg_file[address + 2];
		array_to_datetime(&cfg_file[address], 3, &program->timer.from);
		array_to_datetime(&cfg_file[address], 9, &program->timer.to);
		array_to_cyclogram(&cfg_file[address], 15, &program->timer.cyclogram);
		program->timer.output = cfg_file[address + 22];
		break;
	case RELAY_CONTROL_PROGRAM:
		program->relay.input = cfg_file[address + 2];
		program->relay.constrains = cfg_file[address + 3];
		array_to_datetime(&cfg_file[address], 4, &program->relay.from);
		array_to_datetime(&cfg_file[address], 10, &program->relay.to);
		program->relay.low_bound = array_to_float(&cfg_file[address], 16);
		program->relay.high_bound = array_to_float(&cfg_file[address], 20);
		array_to_cyclogram(&cfg_file[address], 24, &program->relay.cyclogram);
		program->relay.output = cfg_file[address + 31];
		break;
	case PID_CONTROL_PROGRAM:
		program->pid.input = cfg_file[address + 2];
		program->pid.constrains = cfg_file[address + 3];
		array_to_datetime(&cfg_file[address], 4, &program->pid.from);
		array_to_datetime(&cfg_file[address], 10, &program->pid.to);
		program->pid.proportional_gain = array_to_float(&cfg_file[address], 16);
		program->pid.integral_gain = array_to_float(&cfg_file[address], 20);
		program->pid.differential_gain = array_to_float(&cfg_file[address], 24);
		program->pid.output = cfg_file[address + 29];
		break;
	default:
		program->common.type = EMPTY_PROGRAM;
		program->common.id = 0;
		break;
	}
}
