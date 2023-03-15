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
#define ONE_MHZ 1000000

typedef struct
{
	TIM_HandleTypeDef* htim;
	U32 channel;
	U16 buffer[IC_BUF_SIZE];
	U16 averageDeltaTimerTicks;
	U32 timerPeriodNs;
	U32 lastDMAReadValueTimeMs;
	U16 DMA_lastReadValue;
	float conversionRatio;
	float* resultStoreLocation;
	bool stopped;
	bool useVariableSpeedSampling;
	U16 lowResultingValue;
	U16 highResultingValue;
	U16 minSamples;
	U16 timerSize;

} PulseSensor;

void setup_timer_and_start_dma_vss(TIM_HandleTypeDef* htim, U32 channel, U32 timerPeriodNs, float conversionRatio, float* resultStoreLocation, bool useVariableSpeedSampling, U16 lowSamples, U16 highSamples, U16 minSamples);
void setup_timer_and_start_dma(TIM_HandleTypeDef* htim, U32 channel, U32 timerPeriodNs, float conversionRatio, float* resultStoreLocation);
void check_all_dmas();
void checkTransSpeedDMAs();

#endif
