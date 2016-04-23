#include "link.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "cmsis_os.h"
#include "hw.h"

#include "config.h"
#include "io.h"
#include "datetime.h"

#include "modbus.h"

#define LINK_PERIOD_MS 		100

static osThreadId 					link;

#define LINK_PACKET_SIZE 			((uint16_t)255)
static uint8_t 						recv_buffer[LINK_PACKET_SIZE] = {0};
static uint8_t 						send_buffer[LINK_PACKET_SIZE] = {0};

static uint32_t 					ticks;

static struct modbus_instance 		modbus;

static struct adc_input_reg {
	int16_t value[16];
} 									adc_values;


static struct ioslot_value_input_reg {
	float value[60];
} 									ioslot_values;

static const struct modbus_regs_table input_tables[] = {
		{0x0000, sizeof(adc_values) / 2, (uint16_t*)&adc_values},
		{0x1000, sizeof(ioslot_values) / 2, (uint16_t*)&ioslot_values},
		{0, 0, NULL}
};

static struct common_hold_reg {
	struct datetime now;
	uint32_t uptime;
} 									common_values;

static const struct modbus_regs_table holding_tables[] = {
		{0x0000, sizeof(common_values) / 2, (uint16_t*)&common_values},
		{0, 0, NULL}
};

static int modbus_read(struct modbus_instance *instance, uint8_t *buffer, uint8_t max_size)
{
	uart_device_t *dev = (uart_device_t *)instance->arg;
	return device_read(dev, buffer, max_size);
}

static int modbus_write(struct modbus_instance *instance, const uint8_t *packet, uint8_t plen)
{
	uart_device_t *dev = (uart_device_t *)instance->arg;

	device_write(dev, packet, plen);

	MODBUS_RETURN(instance, MODBUS_SUCCESS);
}

#define COILS_OFFSET(__table__, __var__) ((uint8_t*)(&(__table__)) - (uint8_t*)(&(__table__).__var__))
#define COILS_IN(__start__, __offset__,__address__, __count__) ((__start__ + __offset__) >= __address__ && (__start__ + __offset__) <= (__address__ + __count__))

static int modbus_after_write_table(struct modbus_instance *instance, enum modbus_table table, uint16_t address, uint16_t count)
{
	if (table == MODBUS_TABLE_HOLDING_REGISTERS)
	{
		int offset = COILS_OFFSET(common_values, now);
		if (COILS_IN(0x0000, offset, address, count))
		{
			// SYNC CLOCK
		}
	}

	MODBUS_RETURN(instance, MODBUS_SUCCESS);
}

static int modbus_read_file(struct modbus_instance *instance, uint16_t filenum, uint16_t address, uint16_t count, uint8_t *data)
{
	uint16_t start_address;
	bool found = false;

	if (filenum == 0x0001)
	{
		start_address = IOSLOT_START_ADDRESS;
		found = true;
	}

	if (filenum == 0x0002)
	{
		start_address = PROGRAM_START_ADDRESS;
		found = true;
	}

	if (found)
	{
		uint16_t i;
		uint16_t j;

		for (i = 0, j = 0; i < count; ++i, ++address, j += 2)
		{
			uint8_t r[2];
			config_read(start_address + (address << 1), r, 2);

			data[j] = r[1];
			data[j + 1] = r[0];
		}

		MODBUS_RETURN(instance, MODBUS_SUCCESS);
	}

	MODBUS_RETURN(instance, MODBUS_BAD_PARAMS);
}

static int modbus_write_file(struct modbus_instance *instance, uint16_t filenum, uint16_t address, uint16_t count, const uint8_t *data)
{
	uint16_t start_address;
	bool found = false;

	if (filenum == 0x0001)
	{
		start_address = IOSLOT_START_ADDRESS;
		found = true;
	}

	if (filenum == 0x0002)
	{
		start_address = PROGRAM_START_ADDRESS;
		found = true;
	}

	if (found)
	{
		uint16_t i;
		uint16_t j;

		config_lock();

		for (i = 0, j = 0; i < count; ++i, ++address, j += 2)
		{
			uint8_t w[2];

			w[0] = data[j + 1];
			w[1] = data[j];
			config_write(start_address + (address << 1), w, 2);
		}

		config_unlock();

		MODBUS_RETURN(instance, MODBUS_SUCCESS);
	}

	MODBUS_RETURN(instance, MODBUS_BAD_PARAMS);
}

static const struct modbus_functions modbus_fn = {
		/*.open*/ 				NULL,
		/*.write*/				modbus_write,
		/*.read*/				modbus_read,
		/*.close*/				NULL,
		/*.lock*/				NULL,
		/*.before_read_table*/	NULL,
		/*.after_write_table*/	modbus_after_write_table,
		/*.read_file*/			modbus_read_file,
		/*.write_file*/			modbus_write_file
};

static void update_inputs(void)
{
	uint8_t i;

	io_lock();

	for (i = 0; i < 16; ++i)
	{
		adc_values.value[i] = get_adc_value(i);
	}

	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		ioslot_values.value[i] = get_ioslot_value(i);
	}

	io_unlock();

	common_values.now = datetime_now();
	common_values.uptime = ticks;
}

static void _link(void const * argument)
{
	uart_device_t *dev = 0;

	hw_link_init();

	dev = hw_get_link();

	device_open(dev);

	device_setup(dev, UART_DEVICE_PARAMETER_BAUDRATE, 9600);
	device_setup(dev, UART_DEVICE_PARAMETER_BITS, UART_8_BIT);
	device_setup(dev, UART_DEVICE_PARAMETER_STOP_BITS, UART_1_STOP_BIT);
	device_setup(dev, UART_DEVICE_PARAMETER_PARITY_ENABLE, UART_DISABLE_PARITY_BIT);
	device_setup(dev, UART_DEVICE_PARAMETER_PARITY_MODE, 0);
	device_setup(dev, UART_DEVICE_PARAMETER_HALF_DUPLEX, UART_FULL_DUPLEX);

	device_setup(dev, UART_DEVICE_PARAMETER_DMA_ENABLE, UART_DMA_ENABLE);
	device_setup(dev, UART_DEVICE_PARAMETER_RX_TIMEOUT, 22);

	if (!device_configure(dev))
		goto error;

	modbus.arg = dev;
	modbus.address = 0x01;
	modbus.recv_buffer = recv_buffer;
	modbus.recv_buffer_size = LINK_PACKET_SIZE;
	modbus.send_buffer = send_buffer;
	modbus.send_buffer_size = LINK_PACKET_SIZE;

	modbus.id = (uint8_t*)"PLANE";

	modbus.coil_tables = 0;
	modbus.discrete_tables = 0;
	modbus.input_tables = input_tables;
	modbus.holding_tables = holding_tables;

	modbus.functions = &modbus_fn;

	ticks = osKernelSysTick();

	while (1)
	{
		int ret = modbus_io(&modbus);
		if (ret == MODBUS_SUCCESS)
		{
			// OK
		}

		update_inputs();

		osDelayUntil(&ticks, LINK_PERIOD_MS);
	}

	error: osDelay(1); goto error;
}

void link_init(void)
{
	osThreadDef(link, _link, osPriorityAboveNormal, 0, 1024);
	link = osThreadCreate(osThread(link), NULL);
}

