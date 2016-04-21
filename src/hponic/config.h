#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

/* IO SLOTS */

struct common_io_slot /* empty_io_slot */
{
	uint8_t driver;
	uint8_t id;
};

struct analog_input_io_slot
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t num;
	float k;
	float b;
};

struct discrete_input_io_slot
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t pin;
	bool inverse;
};

struct discrete_output_io_slot
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t operation;
	uint8_t pin;
	bool inverse;
};

enum discrete_output_operation
{
	OPERATION_OR = 0,
	OPERATION_AND
};

struct dht22_temperature_io_slot
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t pin;
};

struct dht22_humidity_io_slot
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t pin;
};

union abstract_io_slot
{
	struct common_io_slot 				common;
	struct analog_input_io_slot 		analog_input;
	struct discrete_input_io_slot 		discrete_input;
	struct discrete_output_io_slot 		discrete_output;
	struct dht22_temperature_io_slot 	dht22_temperature;
	struct dht22_humidity_io_slot 		dht22_humidity;
};

enum io_slot_driver
{
	EMPTY_SLOT_DRIVER = 0,
	ANALOG_INPUT_DRIVER,
	DISCRETE_INPUT_DRIVER,
	DISCRETE_OUTPUT_DRIVER,
	DHT22_TEMPERATURE_DRIVER,
	DHT22_HUMIDITY_DRIVER
};

void read_io_slot(uint8_t num, union abstract_io_slot *io_slot);

/* PROGRAMS */

struct common_program /* empty_program */
{
	uint8_t type;
	uint8_t id;
};

struct timer_control_program
{
	uint8_t type;
	uint8_t id;
	
	uint8_t constrains;
	struct datetime from;
	struct datetime to;
	
	struct cyclogram_params cyclogram;
	
	uint8_t output;	
};

struct timer_control_program
{
	uint8_t type;
	uint8_t id;
	
	uint8_t constrains;
	struct datetime from;
	struct datetime to;
	
	struct cyclogram_params cyclogram;
	
	uint8_t output;	
};

struct relay_control_program
{
	uint8_t type;
	uint8_t id;
	
	uint8_t input;
	
	uint8_t constrains;
	struct datetime from;
	struct datetime to;
	
	float low_bound;
	float high_bound;
	
	struct cyclogram_params cyclogram;
	
	uint8_t output;	
};

struct pid_control_program
{
	uint8_t type;
	uint8_t id;
	
	uint8_t input;
	
	uint8_t constrains;
	struct datetime from;
	struct datetime to;
	
	float proportional_gain;
	float integral_gain;
	float differential_gain;
	
	uint8_t output;	
};

union abstract_program
{
	struct common_program 			common;
	struct timer_control_program 	timer;
	struct relay_control_program 	relay;
	struct pid_control_program 		pid;
};

enum program_type
{
	EMPTY_PROGRAM = 0,
	TIMER_CONTROL_PROGRAM,
	RELAY_CONTROL_PROGRAM,
	PID_CONTROL_PROGRAM
};

void read_program(uint8_t num, union abstract_program *program);

void config_init(void);

void config_lock(void);
void config_unlock(void);

void config_read(uint16_d address, uint8_t *data, uint16_t size);
void config_write(uint16_d address, const uint8_t *data, uint16_t size);

#endif /* CONFIG_H_ */
