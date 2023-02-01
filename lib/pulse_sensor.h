// Header guard
#ifndef PULSE_SENSOR_H
#define PULSE_SENSOR_H

#define TIMER_COUNT 4;
#define IC_BUF_SIZE 64;

typedef struct Timer
{
	HAL_timer_typdef* timeR;
	TIM_CHANNEL channel;
	U16 buffer[IC_BUF_SIZE];
	bool stopped;
}

U16 checkTransSpeedDMA(U16* ic1buf, U32* lastDMAReadValueTimeMs, U16 averageSpeedMsPerTick);
