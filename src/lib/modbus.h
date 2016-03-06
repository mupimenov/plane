/*
 * modbus.h
 *
 *  Created on: Feb 27, 2016
 *      Author: mupimenov
 */

#ifndef SRC_LIB_MODBUS_H_
#define SRC_LIB_MODBUS_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef MODBUS_BUFFER_SIZE
#define MODBUS_BUFFER_SIZE 128
#endif

enum modbus_request_function {
	MODBUS_READ_COILS                	= 0x01,
	MODBUS_READ_DISCRETE_INPUTS      	= 0x02,
	MODBUS_READ_HOLDING_REGISTERS    	= 0x03,
	MODBUS_READ_INPUT_REGISTERS      	= 0x04,
	MODBUS_WRITE_SINGLE_COIL         	= 0x05,
	MODBUS_WRITE_SINGLE_REGISTER     	= 0x06,
	MODBUS_READ_EXCEPTION_STATUS     	= 0x07,
	MODBUS_WRITE_MULTIPLE_COILS      	= 0x0F,
	MODBUS_WRITE_MULTIPLE_REGISTERS 	= 0x10,
	MODBUS_REPORT_SLAVE_ID           	= 0x11,
	MODBUS_MASK_WRITE_REGISTER 			= 0x16,
	MODBUS_WRITE_AND_READ_REGISTERS  	= 0x17
};

struct modbus_request {
	uint8_t address;
	enum modbus_request_function function;
	const uint8_t *data;
	uint8_t dlen;
};

enum modbus_answer_error {
	MODBUS_ERROR_OK 					= 0x00,

	MODBUS_ERROR_ILLEGAL_FUNCTION 		= 0x01,
	MODBUS_ERROR_ILLEGAL_DATA_ADDRESS 	= 0x02,
	MODBUS_ERROR_ILLEGAL_DATA_VALUE 	= 0x03,
	MODBUS_ERROR_SERVICE_DEVICE_FAILURE = 0x04,
	MODBUS_ERROR_ACKNOWLEDGE 			= 0x05,
	MODBUS_ERROR_SERVER_DEVICE_BUSY 	= 0x06,
	MODBUS_ERROR_MEMORY_PARITY_ERROR 	= 0x08
};

enum modbus_answer_data_type {
	MODBUS_DATA_TYPE_RAW,
	MODBUS_DATA_TYPE_BIT,
	MODBUS_DATA_TYPE_HALF_WORD
};

struct modbus_answer {
	enum modbus_answer_error error;
	enum modbus_answer_data_type dtype;
	uint8_t *data;
	uint8_t dlen;
};

enum modbus_table {
	MODBUS_TABLE_COILS,
	MODBUS_TABLE_DISCRETE_INPUTS,
	MODBUS_TABLE_HOLDING_REGISTERS,
	MODBUS_TABLE_INPUT_REGISTERS
};

enum modbus_lock_type {
	MODBUS_LOCK,
	MODBUS_UNLOCK
};

struct modbus_bits_table {
	uint16_t address;
	uint16_t count;
	uint8_t *bits;
};

struct modbus_regs_table {
	uint16_t address;
	uint16_t count;
	uint16_t *regs;
};

struct modbus_instance {
	// data
	void *arg;
	uint8_t *recv_buffer;
	uint8_t *send_buffer;
	uint8_t recv_buffer_size;
	uint8_t send_buffer_size;

	// tables
	const struct modbus_bits_table *coil_tables;
	const struct modbus_bits_table *discrete_tables;
	const struct modbus_regs_table *input_tables;
	const struct modbus_regs_table *holding_tables;

	// io
	int (*open)(struct modbus_instance *instance);
	int (*write)(struct modbus_instance *instance, const uint8_t *packet, uint8_t plen);
	int (*read)(struct modbus_instance *instance, uint8_t *buffer, uint8_t max_size);
	int (*close)(struct modbus_instance *instance);

	// locks
	int (*lock)(struct modbus_instance *instance, enum modbus_table table, enum modbus_lock_type lock_);

	// hooks
	int (*before_read_table)(struct modbus_instance *instance, enum modbus_table table, uint16_t address, uint16_t count);
	int (*after_write_table)(struct modbus_instance *instance, enum modbus_table table, uint16_t address, uint16_t count);

	// id
	uint8_t *id;
	// address
	uint8_t address;
	// status
	uint32_t error;
};

enum modbus_instance_error {
	MODBUS_SUCCESS = 0,
	MODBUS_INVAL,
	MODBUS_DEVICE_ERROR,
	MODBUS_NO_PACKET,
	MODBUS_NOT_IMPLEMENTED,
	MODBUS_BAD_CRC,
	MODBUS_BAD_ADDRESS,
	MODBUS_BAD_COMMAND,
	MODBUS_BAD_PARAMS,
	MODBUS_INT

};

#define MODBUS_RETURN(__instance__, __err__) { (__instance__)->error = __err__; return (__err__? -1: 0); }

int modbus_io(struct modbus_instance *instance);
int modbus_send_error(struct modbus_instance *instance, uint8_t function, uint8_t error);

struct modbus_read_command {
	uint16_t address;
	uint16_t count;
};

int modbus_parse_read_command(const uint8_t *data, uint8_t dlen, struct modbus_read_command *command);

int modbus_read_coils_cmd(struct modbus_instance *instance, struct modbus_request *req);
int modbus_read_discrete_inputs_cmd(struct modbus_instance *instance, struct modbus_request *req);

int modbus_read_holding_registers_cmd(struct modbus_instance *instance, struct modbus_request *req);
int modbus_read_input_registers_cmd(struct modbus_instance *instance, struct modbus_request *req);

struct modbus_write_bit_command {
	uint16_t address;
	uint16_t data;
};

int modbus_parse_write_bit_command(const uint8_t *data, uint8_t dlen, struct modbus_write_bit_command *command);

struct modbus_write_reg_command {
	uint16_t address;
	uint16_t data;
};

int modbus_parse_write_reg_command(const uint8_t *data, uint8_t dlen, struct modbus_write_reg_command *command);

int modbus_write_coil_cmd(struct modbus_instance *instance, struct modbus_request *req);
int modbus_write_register_cmd(struct modbus_instance *instance, struct modbus_request *req);

struct modbus_write_multiple_command {
	uint16_t address;
	uint16_t count;
	uint8_t bytes;
};

int modbus_parse_write_multiple_command(const uint8_t *data, uint8_t dlen, struct modbus_write_multiple_command *command);

int modbus_write_multiple_coils_cmd(struct modbus_instance *instance, struct modbus_request *req);
int modbus_write_multiple_regs_cmd(struct modbus_instance *instance, struct modbus_request *req);

int modbus_report_slave_id_cmd(struct modbus_instance *instance, struct modbus_request *req);

struct modbus_mask_write_command {
	uint16_t address;
	uint16_t and_mask;
	uint16_t or_mask;
};

int modbus_parse_mask_write_reg_command(const uint8_t *data, uint8_t dlen, struct modbus_mask_write_command *command);

int modbus_mask_write_reg_cmd(struct modbus_instance *instance, struct modbus_request *req);

struct modbus_write_read_regs_command {
	uint16_t read_address;
	uint16_t read_count;
	uint16_t write_address;
	uint16_t write_count;
	uint8_t bytes;
};

int modbus_parse_write_read_regs_command(const uint8_t *data, uint8_t dlen, struct modbus_write_read_regs_command *command);

int modbus_write_read_regs_cmd(struct modbus_instance *instance, struct modbus_request *req);

#endif /* SRC_LIB_MODBUS_H_ */
