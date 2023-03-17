#include "stm32f4xx_hal.h"
#include "base_types.h"
#include <stdbool.h>

// Header guard
#ifndef PULSE_SENSOR_H
#define PULSE_SENSOR_H

#define TIMER_COUNT 4
#define IC_BUF_SIZE 64
#define MS_IN_A_MINUTE 60000
#define DMA_STOPPED_TIMEOUT_MS 200
#define ONE_MHZ 1000000

typedef struct
{
	TIM_HandleTypeDef* htim; 		// Timer that was used to set up input capture and DMA on
	U32 channel;					// Channel of the given timer
	float timerPeriodSeconds;		// Period of timer ticks (they can be different between different timers)
	float conversionRatio;			// Number to multiple the frequency by to get the desired result
	float* resultStoreLocation;		// Float pointer to a location you want the resulting RPM to update
	bool useVariableSpeedSampling;	// Bool for weather or not to vary the amount of samples taken from the buffer to determine the resulting RPM average
	U16 lowPulsesPerSecond;			// Value at which the min samples will be used from the buffer behind the DMA position. Set to desired number of samples if not using variable speed sampling, or 0 for max samples
	U16 highPulsesPerSecond;			// Value at which all 64 values in the buffer are sampled to get the resulting speed value. Set to 0 if not using variable speed sampling
	U16 minSamples;					// Minimum amount of samples to take if using variable speed sampling

	U16 timerSize;
	U32 buffer[IC_BUF_SIZE];

	U16 averageDeltaTimerTicks;
	U32 lastDMAReadValueTimeMs;
	U16 DMA_lastReadValue;

	bool stopped;

} PulseSensor;

void setup_timer_and_start_dma_vss(TIM_HandleTypeDef* htim, U32 channel, U32 timerPeriodNs, float conversionRatio, float* resultStoreLocation, bool useVariableSpeedSampling, U16 lowPulsesPerSecond, U16 highPulsesPerSecond, U16 minSamples);
void setup_timer_and_start_dma(TIM_HandleTypeDef* htim, U32 channel, U32 timerPeriodNs, float conversionRatio, float* resultStoreLocation);
void check_all_dmas();
void check_timer_dma(int sensorNumber);

#endif
