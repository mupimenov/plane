/*
 * task_link.c
 *
 *  Created on: Feb 23, 2016
 *      Author: mupimenov
 */

#include "task_link.h"

#include <string.h>
#include <stdio.h>

#include "cmsis_os.h"
#include "hw.h"

#include "task_measure.h"

#include "modbus.h"

#define GRAVITY_ACC 				((float)0.00980665f)

#define LINK_TASK_PERIOD_MS 		100

static osThreadId task_link;

#define LINK_PACKET_SIZE 			((uint16_t)255)
static uint8_t 						lnk_recv_buffer[LINK_PACKET_SIZE] = {0};
static uint8_t 						lnk_send_buffer[LINK_PACKET_SIZE] = {0};

static uint32_t 					lnk_ticks;

static struct modbus_instance 		lnk_modbus;

static struct discrete_inputs_table {
	uint8_t dummy;
} 									lnk_discrete1;

static struct input_registers_table {
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t compass_x;
	int16_t compass_y;
	int16_t compass_z;
	int16_t pitch;
	int16_t roll;
	int16_t yaw;
} 									lnk_input1;

static struct coils_table {
	uint8_t dummy;
} 									lnk_coil1;

static struct holding_registers_table {
	uint16_t request_count;
	uint16_t measuring_duration;
	uint16_t reserved2;
	uint16_t reserved3;

} 									lnk_holding1;

static const struct modbus_bits_table lnk_discrete_tables[] = {
		{0x0000, sizeof(lnk_discrete1), (uint8_t*)&lnk_discrete1},
		{0, 0, NULL}
};

static const struct modbus_regs_table lnk_input_tables[] = {
		{0x0000, sizeof(lnk_input1) / 2, (uint16_t*)&lnk_input1},
		{0, 0, NULL}
};

static const struct modbus_bits_table lnk_coil_tables[] = {
		{0x0000, sizeof(lnk_coil1), (uint8_t*)&lnk_coil1},
		{0, 0, NULL}
};

static const struct modbus_regs_table lnk_holding_tables[] = {
		{0x0000, sizeof(lnk_holding1) / 2, (uint16_t*)&lnk_holding1},
		{0, 0, NULL}
};

static int _modbus_read(struct modbus_instance *instance, uint8_t *buffer, uint8_t max_size)
{
	uart_device_t *dev = (uart_device_t *)instance->arg;
	return device_read(dev, 0, buffer, max_size);
}

static int _modbus_write(struct modbus_instance *instance, const uint8_t *packet, uint8_t plen)
{
	uart_device_t *dev = (uart_device_t *)instance->arg;

	device_write(dev, 0, packet, plen);

	MODBUS_RETURN(instance, MODBUS_SUCCESS);
}

static void _update_inputs(void)
{
	float data[3];

	get_gyro_data(data);

	lnk_input1.gyro_x = (int16_t)data[0];
	lnk_input1.gyro_y = (int16_t)data[1];
	lnk_input1.gyro_z = (int16_t)data[2];

	get_accel_data(data);

	lnk_input1.accel_x = (int16_t)(data[0]);
	lnk_input1.accel_y = (int16_t)(data[1]);
	lnk_input1.accel_z = (int16_t)(data[2]);

	get_angle_data(data);

	lnk_input1.pitch = (int16_t)(data[0] * 180.0f / 3.14f);
	lnk_input1.roll = (int16_t)(data[1] * 180.0f / 3.14f);
	lnk_input1.yaw = (int16_t)(data[2] * 180.0f / 3.14f);

	lnk_holding1.measuring_duration = get_measuring_duration();
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

	lnk_modbus.arg = dev;
	lnk_modbus.address = 0x01;
	lnk_modbus.recv_buffer = lnk_recv_buffer;
	lnk_modbus.recv_buffer_size = LINK_PACKET_SIZE;
	lnk_modbus.send_buffer = lnk_send_buffer;
	lnk_modbus.send_buffer_size = LINK_PACKET_SIZE;

	lnk_modbus.id = (uint8_t*)"PLANE";

	lnk_modbus.coil_tables = lnk_coil_tables;
	lnk_modbus.discrete_tables = lnk_discrete_tables;
	lnk_modbus.input_tables = lnk_input_tables;
	lnk_modbus.holding_tables = lnk_holding_tables;

	lnk_modbus.open = NULL;
	lnk_modbus.close = NULL;
	lnk_modbus.read = _modbus_read;
	lnk_modbus.write = _modbus_write;

	lnk_modbus.lock = NULL;

	lnk_modbus.after_write_table = NULL;
	lnk_modbus.before_read_table = NULL;

	lnk_ticks = osKernelSysTick();

	while (1)
	{
		int ret = modbus_io(&lnk_modbus);
		if (ret == 0)
		{
			// OK
			lnk_holding1.request_count++;
		}

		_update_inputs();

		osDelayUntil(&lnk_ticks, LINK_TASK_PERIOD_MS);
	}

	error: osDelay(1); goto error;
}

void task_link_init(void)
{
	osThreadDef(link_task, _link, osPriorityAboveNormal, 0, 1024);
	task_link = osThreadCreate(osThread(link_task), NULL);
}
