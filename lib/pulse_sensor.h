#include "stm32f4xx_hal.h"
#include "base_types.h"
#include <stdbool.h>

// Header guard
#ifndef PULSE_SENSOR_H
#define PULSE_SENSOR_H

// #define DEBUG_PS

#define TIMER_COUNT 4
#define IC_BUF_SIZE 128
#define MAX_DELTAS 64
#define MS_IN_A_MINUTE 60000
#define ONE_MHZ 1000000
#define DUPLICATE_VALUE_TICK_DIFFERENCE 25	// TODO: See if there's a universal value or if needs to be set individually

// Error codes - may not be necessary
#define CANNOT_DETECT_TIMER_SIZE -1
#define READ_SUCCESS 0
#define NO_NEW_VALUE 1
#define STALE_BUFFER_WIPE 2
#define STOPPED 3
#define UN_STOPPED 4
#define INF_OR_NAN_RESULT 5

typedef struct
{
	// Passed in values
	TIM_HandleTypeDef* htim; 		// Timer that was used to set up input capture and DMA on
	U32 channel;					// Channel of the given timer
	float timerPeriodSeconds;		// Period of timer ticks (they can be different between different timers)
	float conversionRatio;			// Number to multiple the frequency by to get the desired result
	float* resultStoreLocation;		// Float pointer to a location you want the resulting RPM to update
	U16 dmaStoppedTimeoutMS;		// Max number of miliseconds between pulses before rotating object should be declared to have stopped. 16hz min for
	bool useVariableSpeedSampling;	// Bool for weather or not to vary the amount of samples taken from the buffer to determine the resulting RPM average
	U16 lowPulsesPerSecond;			// Value at which the min samples will be used from the buffer behind the DMA position. Set to desired number of samples if not using variable speed sampling, or 0 for max samples
	U16 highPulsesPerSecond;		// Value at which all 64 values in the buffer are sampled to get the resulting speed value. Set to 0 if not using variable speed sampling
	U16 minSamples;					// Minimum amount of samples to take if using variable speed sampling

	U32 buffer[IC_BUF_SIZE];

	// Stored values
	float vssSlope;
	U32 lastDMAReadValueTimeMs;
	U16 timerSize;
	U16 DMALastReadValue;
	bool stopped;

	//Debug/tracking variables
#ifdef DEBUG_PS
	float averageDeltaTimerTicks;
	U16 numSamples;
	U16 numDroppedValues;
	U16 numDuplicateValues;
	U16 DMACurrentPosition;
#endif

} PulseSensor;

int setup_pulse_sensor_vss(TIM_HandleTypeDef* htim, U32 channel, float conversionRatio, float* resultStoreLocation, U16 dmaStoppedTimeoutMS, bool useVariableSpeedSampling, U16 lowPulsesPerSecond, U16 highPulsesPerSecond, U16 minSamples);
int setup_pulse_sensor(TIM_HandleTypeDef* htim, U32 channel, float conversionRatio, float* resultStoreLocation, U16 dmaStoppedTimeoutMS);
int check_pulse_sensors();
int evaluate_pulse_sensor(int sensorNumber);

#endif
