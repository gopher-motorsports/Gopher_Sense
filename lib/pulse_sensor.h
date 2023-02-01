// Header guard
#ifndef PULSE_SENSOR_H
#define PULSE_SENSOR_H

#define TIMER_COUNT 4
#define IC_BUF_SIZE 64
#define MS_IN_A_MINUTE 60000

typedef struct PulseSensor
{
	HAL_timer_typdef* htim;
	TIM_CHANNEL channel;
	U16 buffer[IC_BUF_SIZE];
	U16 averageSpeedTimerTicks = 0;
	U32 timerPeriodNs = 0;
	U32 lastDMACheckMs = 0;
	U32 lastDMAReadValueTimeMs = 0;
	U16 DMA_lastReadValue = 0;
	U16 lowSamples = 0;
	U16 highSamples = 0;
	U16 ticksPerRev = 0;
	float conversionRatio = 0;
	float* resultStoreLocation;
	bool useVariableSpeedSampling = false;
	bool stopped = false;
}

void setupTimerAndStartDMA(HAL_timer_typdef* htim, TIM_CHANNEL channel, U32 timerPeriodNs, U16 lowSamples, U16 highSamples, U16 ticksPerRev, float conversionRatio, float* resultStoreLocation, bool useVariableSpeedSampling);
U16 checkTransSpeedDMAs();
