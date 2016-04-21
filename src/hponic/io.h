#ifndef IO_H_
#define IO_H_

#include <stdint.h>

#define IOSLOTS_COUNT 60

void io_init(void);
void io_execute_in(void);
void io_execute_out(void);

uint8_t input_discrete(uint8_t id, int *err);
float input_analog(uint8_t id, int *err);

void output_discrete(uint8_t id, uint8_t value);

#endif /* IO_H_ */
