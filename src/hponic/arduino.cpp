#include "arduino.h"

#include "cmsis_os.h"

unsigned long millis(void)
{
	uint32_t ticks = osKernelSysTick();
	return ticks;
}
