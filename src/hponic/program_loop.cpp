#include "program_loop.h"

#include "cmsis_os.h"

#include "config.h"
#include "io.h"
#include "program.h"

#include "hw.h"

#define PROG_LOOP_PERIOD_MS 	100

static osThreadId 				prog_loop;
static uint32_t 				ticks;

static void update_common_values(void)
{
	struct rtc_date d = hw_rtc_get_date();
	struct rtc_time t = hw_rtc_get_time();
	struct datetime dt;
	dt.hours = t.hours;
	dt.minutes = t.minutes;
	dt.seconds = t.seconds;
	dt.day = d.day;
	dt.month = d.month;
	dt.year = d.year;

	datetime_set(&dt);
}

static void _prog_loop(void const * argument)
{
	ticks = osKernelSysTick();

	while (1)
	{
		update_common_values();

		io_execute_in();
		program_execute();
		io_execute_out();

		osDelayUntil(&ticks, PROG_LOOP_PERIOD_MS);
	}
}

void program_loop_init(void)
{
	config_init();
	io_init();
	program_init();

	osThreadDef(prog_loop, _prog_loop, osPriorityAboveNormal, 0, 1024);
	prog_loop = osThreadCreate(osThread(prog_loop), NULL);
}
