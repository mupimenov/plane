/*
 * task_led.c
 *
 *  Created on: Feb 23, 2016
 *      Author: mupimenov
 */

#include "task_led.h"

#include "cmsis_os.h"

#include "hw.h"

static osThreadId task_led;

static void _led(void const * argument)
{
	char state = 0;

	hw_led_init();

	while (1)
	{
		state ^= 1;

		hw_led_set_green(state);

		osDelay(500);
	}
}

void task_led_init(void)
{
	osThreadDef(led_task, _led, osPriorityAboveNormal, 0, 256);
	task_led = osThreadCreate(osThread(led_task), NULL);
}
