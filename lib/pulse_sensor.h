#include "stm32f4xx_hal.h"
#include "base_types.h"
#include <stdbool.h>

// Header guard
#ifndef PULSE_SENSOR_H
#define PULSE_SENSOR_H

#define TIMER_COUNT 4
#define IC_BUF_SIZE 64
#define MS_IN_A_MINUTE 60000
#define DMA_STOPPED_TIMEOUT_MS 10

typedef struct
{
	TIM_HandleTypeDef* htim;
	U32 channel;
	U16 buffer[IC_BUF_SIZE];
	U16 averageSpeedTimerTicks;
	U32 timerPeriodNs;
	U32 lastDMAReadValueTimeMs;
	U16 DMA_lastReadValue;
	float conversionRatio;
	float* resultStoreLocation;
	bool stopped;
	bool useVariableSpeedSampling;
	U16 lowTimeDeltaValue;
	U16 highTimeDeltaValue;
} PulseSensor;

void setupTimerAndStartDMA(TIM_HandleTypeDef* htim, U32 channel, U32 timerPeriodNs, float conversionRatio, float* resultStoreLocation, bool useVariableSpeedSampling, U16 lowSamples, U16 highSamples);
void checkTransSpeedDMAs();

#endif
