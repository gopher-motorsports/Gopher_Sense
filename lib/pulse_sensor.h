#include "main.h"
#include "base_types.h"
#include <stdbool.h>
#include <math.h>

// Header guard
#ifndef PULSE_SENSOR_H
#define PULSE_SENSOR_H

//#define DEBUG_PS

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

#define CALCULATE_MPH_CONVERSION_RATIO(ticksPerRev, radiusInches) (radiusInches*2*M_PI/ticksPerRev*60*60/5280) // Circumpherence/ticksPerRev*minutes*hours/inchesPerMile

typedef struct
{
	// Passed in values
	TIM_HandleTypeDef* htim; 		// Timer that was used to set up input capture and DMA on
	U32 channel;					// Channel of the given timer
	U8 hdmaChannel;					// [Hopefully temporary] Value of the array position of the timers hdma the selected timer and channel will use - needed for getting the dma position
	float conversionRatio;			// Number to multiple the frequency by to get the desired result
	float* resultStoreLocation;		// Float pointer to a location you want to be updated after using check_pulse_sensor()
	U16 dmaStoppedTimeoutMS;		// Max number of miliseconds between pulses before rotating object should be declared to have stopped
	bool useVariableSpeedSampling;	// Boolean for weather or not to vary the amount of samples taken from the buffer to calculate a result
	U16 lowPulsesPerSecond;			// Value at which the min samples will be used from the buffer behind the DMA position
	U16 highPulsesPerSecond;		// Value at which the max number of deltas from the buffer will sampled to get the resulting speed value
	U16 minSamples;					// Minimum amount of samples to take if using variable speed sampling

	U32 buffer[IC_BUF_SIZE];

	// Stored values
	float vssSlope;
	float timerPeriodSeconds;		// Period of timer ticks (can be different between different timers) - current found automatically in sensor init
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

int setup_pulse_sensor_vss(TIM_HandleTypeDef* htim, U32 channel, U8 hdmaChannel, float conversionRatio, float* resultStoreLocation, U16 dmaStoppedTimeoutMS, bool useVariableSpeedSampling, U16 lowPulsesPerSecond, U16 highPulsesPerSecond, U16 minSamples);
int setup_pulse_sensor(TIM_HandleTypeDef* htim, U32 channel, U8 hdmaChannel, float conversionRatio, float* resultStoreLocation, U16 dmaStoppedTimeoutMS);
int check_pulse_sensors();
int evaluate_pulse_sensor(int sensorNumber);

#endif
