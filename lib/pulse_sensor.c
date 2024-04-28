/**
 * Library for operating pulse one or multiple sensors and returning an result through a float pointer.
 * Currently needs to be a f4 stm32 for auto-detecting the hdma array value
 *
 * Alex Tong
 */

#include "pulse_sensor.h"
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static PulseSensor pulseSensor[TIMER_COUNT] = {0};
static U8 numSensors = 0;

static void clear_buffer_and_reset_dma(U8 sensorNumber);
static float convert_delta_time_to_frequency(float deltaTime, float timerPeriodSeconds);

// Most useful debug variables
#ifdef DEBUG_PS
static U32 deltaList[IC_BUF_SIZE] = {0};
static float TrackedFrequency = 0;
static U32 timeBetween = 100;
static U32 lastTime = 0;
#endif

/**
 * Function for setting up timer with all details, use this function directly to include variable speed sampling (vss) data.
 *
 * him - Timer that was used to set up input capture and DMA on (Ex. &htim2)
 * channel - Channel of the given timer (Ex. TIM_CHANNEL_1) - NOTE: You need to be very aware if different timer channels cannot be run at the same time. For example on timer 2, Channel 4 and channel 2 cannot be used at the same time as they share the same hdma positions (below) and fail to start if the other is configured first. You can tell be looking at the channels that you're setting up when configuring for DMA - if the channel you're trying to set up says somthing like TIM2_CH2/CH4, this means they're interchangeable. You may have to look if there is a different timer availible on the same pins (In this case tim5 was availible which has no channel conflicts.
 * hdmaChannel - // [Hopefully temporary] Value of the array position of the timers hdma the selected timer and channel will use - needed for getting the dma position
 * conversionRatio - Number to multiple the frequency by to get the desired result
 * 		(Ex. for TCM: (X pulses / 1 sec) * (60 sec / 1 min) * (1 rev / 30 pulses) = RPM, so conversion ration would be 60/30 = 2)
 * resultStoreLocation - Float pointer to a location you want to be updated after using check_pulse_sensor() (Ex. &(tcm_data.trans_speed)
 * dmaStoppedTimeoutMS - Max number of miliseconds between pulses before rotating object should be declared to have stopped.
 * 		Different timers/configurations may have different limits to this value before the buffer may roll over or the deltas get too big. (Ex. 60 for 60 miliseconds)
 * useVariableSpeedSampling - Boolean for weather or not to vary the amount of samples taken from the buffer to calculate a result. Expected to be true when using setup_pulse_sensor_vss(), otherwise use setup_pulse_sensor()
 * lowPulsesPerSecond - Value at which the min samples will be used from the buffer behind the DMA position
 * highPulsesPerSecond - Value at which the max number of deltas from the buffer will sampled to get the resulting speed value
 * minSamples - Minimum amount of samples to take if using variable speed sampling
 * maxSamples - Maxium amount of samples to take - 64 recommended high, max of 100 (will not hit if there is duplicate values)
 */
int setup_pulse_sensor_vss_freq(
		TIM_HandleTypeDef* htim,
		U32 channel,
		float conversionRatio,
		float* resultStoreLocation,
		U16 dmaStoppedTimeoutMS,
		bool useVariableSpeedSampling,
		U16 lowPulsesPerSecond,
		U16 highPulsesPerSecond,
		U16 minSamples,
		U16 maxSamples,
		float* frequencyStoreLocation
		)
{
	PulseSensor* newSensor = &pulseSensor[numSensors];

	if (maxSamples > MAX_READ_BUFFER_VALUES) return MAX_VALUES_TOO_HIGH; // Max samples is too low or too high
	if (useVariableSpeedSampling) {
		if (lowPulsesPerSecond == 0) return TOO_LOW_LOW_PPS;	// Low pulses per second is too low or too high
		if (highPulsesPerSecond == 0) return TOO_LOW_HIGH_PPS;	// High pulses per second is too low or too high
		if (minSamples == 0 || minSamples > MAX_READ_BUFFER_VALUES) return TOO_LOW_OR_HIGH_MIN_SAMPLES;	// Min samples is too low or too high
		if (maxSamples <= minSamples) return MAX_SAMPLES_NOT_GREATER_THAN_MIN; // Max samples is not greater than min samples
	}

	// Set the local values of the pulse sensor data struct to the given parameters
	newSensor->htim = htim;
	newSensor->channel = channel;
	newSensor->conversionRatio = conversionRatio;
	newSensor->resultStoreLocation = resultStoreLocation;
	newSensor->useVariableSpeedSampling = useVariableSpeedSampling;
	newSensor->dmaStoppedTimeoutMS = dmaStoppedTimeoutMS;
	newSensor->lowPulsesPerSecond = lowPulsesPerSecond;
	newSensor->highPulsesPerSecond = highPulsesPerSecond;
	newSensor->minSamples = minSamples;
	newSensor->maxSamples = maxSamples;
	newSensor->frequencyStoreLocation = frequencyStoreLocation;

	U32 clockFrequency = HAL_RCC_GetSysClockFreq() * 0.5; // Multiply by 0.5 as timers are 1/2 the clock frequency

	// Evaluate the size of the given timer by checking if the Auto reload value is the max size of a 16bit value 0xFFFF
	// TODO: Change clock frequency if certain timers
	if (htim->Instance->ARR == 0xFFFF) {
		newSensor->timerSize = 16;
	} else if (htim->Instance->ARR == 0xFFFFFFFF) {
		newSensor->timerSize = 32;
	} else {
		return CANNOT_DETECT_TIMER_SIZE;
	}

	switch(channel) {
		case TIM_CHANNEL_1:
			newSensor->hdmaChannel = 1;
			break;
		case TIM_CHANNEL_2:
			newSensor->hdmaChannel = 2;
			break;
		case TIM_CHANNEL_3:
			newSensor->hdmaChannel = 3;
			break;
		case TIM_CHANNEL_4:
			newSensor->hdmaChannel = 4;
			break;
		default:
			return CANNOT_DETECT_HDMA_CHANNEL;
			break;
	}

	newSensor->timerPeriodSeconds = 1 / ((float)clockFrequency / (htim->Instance->PSC + 1));

	// Clear the sensor's buffer
	memset(newSensor->buffer, 0, sizeof(U32)*IC_BUF_SIZE);

	// Default other values to 0
	newSensor->lastDMAReadValueTimeMs = 0;
	newSensor->DMALastReadValue = 0;
	newSensor->stopped = true;
	newSensor->vssSlope = (maxSamples - minSamples) / (float)(highPulsesPerSecond - lowPulsesPerSecond); // Calculate the constant slope value for vss once at the beginning here.

	if(HAL_TIM_IC_Start_DMA(htim, channel, (U32*)(pulseSensor[numSensors].buffer), IC_BUF_SIZE)) {
		return STARTING_DMA_FAILED;
	}

	numSensors++;

	return NO_PULSE_SENSOR_ISSUES;
}

// Function for setting up timer without variable speed sampling (vss)
int setup_pulse_sensor(
		TIM_HandleTypeDef* htim,
		U32 channel,
		float conversionRatio,
		float* resultStoreLocation,
		U16 dmaStoppedTimeoutMS,
		U16 numSamples
		)
{
	return setup_pulse_sensor_vss_freq(
		htim,
		channel,
		conversionRatio,
		resultStoreLocation,
		dmaStoppedTimeoutMS,
		false,
		0,	// Low RPM Value
		0,	// High RPM Value
		0, 	// Min samples
		numSamples,
		0
		);
}

// Function for setting up timer with variable speed sampling (vss), specified differently for backwards compatibility.
int setup_pulse_sensor_vss(
		TIM_HandleTypeDef* htim,
		U32 channel,
		float conversionRatio,
		float* resultStoreLocation,
		U16 dmaStoppedTimeoutMS,
		bool useVariableSpeedSampling,
		U16 lowPulsesPerSecond,
		U16 highPulsesPerSecond,
		U16 minSamples,
		U16 maxSamples
		)
{
	return setup_pulse_sensor_vss_freq(
		htim,
		channel,
		conversionRatio,
		resultStoreLocation,
		dmaStoppedTimeoutMS,
		useVariableSpeedSampling,
		lowPulsesPerSecond,	// Low RPM Value
		highPulsesPerSecond,	// High RPM Value
		minSamples, 	// Min samples
		maxSamples,
		0
		);
}

// Function for going through all of the currently set up pulse sensors
int check_pulse_sensors() {
	bool success = true;
	for (int sensorNumber = 0; sensorNumber < numSensors; sensorNumber++) {
		pulseSensor[sensorNumber].errorCode = evaluate_pulse_sensor(sensorNumber);
		if(pulseSensor[sensorNumber].errorCode < 0) {
			success = false;
		}
	}

	return success ? NO_PULSE_SENSOR_ISSUES : -1;
}

// Function that goes through the buffer of the given pulse sensor, handling edge cases, and setting the return value to the found speed value.
int evaluate_pulse_sensor(int sensorNumber) {

	// Get the current tick and use it throughout the whole function so it can't change between operations.
	U32 currentTick = HAL_GetTick();

	// Find the current position of DMA for the current sensor
	// TODO: This NEEDS to be updated for deadling with other times and channels than the TCM - hdma[1] doesn't always work
	S16 DMACurrentPosition = IC_BUF_SIZE - (U16)((pulseSensor[sensorNumber].htim->hdma[pulseSensor[sensorNumber].hdmaChannel])->Instance->NDTR);

	// Find the value in question, which is 1 position backwards from the DMA's current position, which is the end of the buffer copy.
	S16 lastBufferPosition = (DMACurrentPosition - 1 + IC_BUF_SIZE) % IC_BUF_SIZE;

	U32 valueInQuestion = pulseSensor[sensorNumber].buffer[lastBufferPosition];

	if (pulseSensor[sensorNumber].stopped) {	// If we already know we're stopped, check if we should end early
		if (valueInQuestion != 0) {	// If we were previously stopped but may be moving again.
			// Check if this is just really a slow and occasional value
			if (valueInQuestion == pulseSensor[sensorNumber].DMALastReadValue) {
				// We're still seeing the same value, is it old now? We can't get here if it was last a 0.
				if (currentTick - pulseSensor[sensorNumber].lastDMAReadValueTimeMs >= pulseSensor[sensorNumber].dmaStoppedTimeoutMS) {
					// Clear the random values that aren't fast enough so they don't sit and become stale
					clear_buffer_and_reset_dma(sensorNumber);
					pulseSensor[sensorNumber].DMALastReadValue = 0; // Set to 0 so there aren't accidents.
				}
				return STALE_BUFFER_WIPE;
			} else {
				// TODO see if this should be changed from min samples
				if (pulseSensor[sensorNumber].buffer[(DMACurrentPosition - 1 - pulseSensor[sensorNumber].minSamples + IC_BUF_SIZE) % IC_BUF_SIZE] != 0) {
					// The value is new, it didn't get wiped, and the last wasn't 0 so we're good again
					pulseSensor[sensorNumber].stopped = false; // Declare we are no longer stopped and move on.

#ifdef DEBUG_PS
			printf("*** UN-STOPPED ***\n");
#endif
				} else {
					// A new unwiped value but it's just the first after 0. Log it and go again.
					pulseSensor[sensorNumber].DMALastReadValue = valueInQuestion;
					pulseSensor[sensorNumber].lastDMAReadValueTimeMs = currentTick;
					return UN_STOPPED;
				}
			}
		} else {
			*(pulseSensor[sensorNumber].resultStoreLocation) = 0; // Otherwise make sure the store location value is 0 and leave.
			return NO_NEW_VALUE;
		}
	}

	if (pulseSensor[sensorNumber].DMALastReadValue == valueInQuestion) {	// Check if the last read value is the same as the current
		if (currentTick - pulseSensor[sensorNumber].lastDMAReadValueTimeMs >= pulseSensor[sensorNumber].dmaStoppedTimeoutMS){	// Check if we haven't changed values in a while which might mean we're stopped
			pulseSensor[sensorNumber].stopped = true;

			// Clear buffer so any non-zero values will be quickly identified that the car is moving again. This will also reset the DMA position.
			clear_buffer_and_reset_dma(sensorNumber);

			pulseSensor[sensorNumber].DMALastReadValue = 0;
			pulseSensor[sensorNumber].lastDMAReadValueTimeMs = currentTick;
			*(pulseSensor[sensorNumber].resultStoreLocation) = 0;

#ifdef DEBUG_PS
			printf("=== STOPPED ===\n");
#endif

			return STOPPED;
		}
		// Value is the same as the last but it hasn't been enough time to declare stale, hop out of here for now because there's nothing new to process
		return NO_NEW_VALUE;
	}


	// ===== Passed All Breakpoint Checks =====
	pulseSensor[sensorNumber].DMALastReadValue = valueInQuestion;
	pulseSensor[sensorNumber].lastDMAReadValueTimeMs = currentTick;

	// Declare amount of samples and evaluate initial size based on if we're using variable sampling. If we are we'll determine it again after some deltas are evaluated
	U16 numSamples = pulseSensor[sensorNumber].useVariableSpeedSampling ? pulseSensor[sensorNumber].minSamples + 1 : pulseSensor[sensorNumber].maxSamples;

	// Calculate the deltas between each of the time values and store them in deltaTotal
	U32 deltaTotal = 0; // Gets big adding up all U32 deltas but not big enough to need U64 before stopped if like <1 full second
	U32 lastDelta = 0;
	U32 last2ndDelta = 0;
	U16 numDeltas = 0;
	U32 delta = 0;
	U16 i = 0;

	// Begin by going through the first minimum amount samples to determine how many we should read, and then find the average delta while rejecting any dropped DMA values
	// While loop till we either have enough deltas or have passed max readings we can take
	while ((numDeltas < numSamples) && (i < MAX_READ_BUFFER_VALUES)) {
		U32 value1 = pulseSensor[sensorNumber].buffer[(DMACurrentPosition - 1 - i + IC_BUF_SIZE) % IC_BUF_SIZE];
		U32 value2 = pulseSensor[sensorNumber].buffer[(DMACurrentPosition - 2 - i + IC_BUF_SIZE) % IC_BUF_SIZE];
		if (value1 != 0 && value2 != 0) { // Check if 0s because of previously empty buffer, because otherwise a value would simply be a time amount
			// Check if the difference is so small it can be considered a duplicate of the last
			if (abs(value2 - value1) > DUPLICATE_VALUE_TICK_DIFFERENCE) {
				if (value1 < value2) {
					delta = (29999 + value1) - value2; // Buffer roll-over (timer resets to 0) protection
				} else {
					delta = value1 - value2; // Buffer roll-over (timer resets to 0) protection
				}
#ifdef DEBUG_PS
				// Store deltas in an array for  if we're debugging
				deltaList[numDeltas] = delta;
#endif
				// Begin smoothing for issue where DMA loses values
				if(numDeltas == 2) { // On the 3rd delta check if the first value (unprotected by main checker) was an extra large (due to dropped value) that needs to be overwritten
					if((last2ndDelta > lastDelta * 1.8) && (last2ndDelta > delta * 1.8)) {
						deltaTotal -= lastDelta;
						deltaTotal += delta;
#ifdef DEBUG_PS
						pulseSensor[sensorNumber].numDroppedValues++;
#endif
					}
				} else if (numDeltas > 2) { // Make sure not to check first 2
					if ((lastDelta > delta * 1.8) && (lastDelta > last2ndDelta * 1.8)) {
						deltaTotal -= lastDelta;
						lastDelta = (delta + last2ndDelta) * 0.5;
						deltaTotal += lastDelta;
#ifdef DEBUG_PS
						pulseSensor[sensorNumber].numDroppedValues++;
#endif
					}
				}
				deltaTotal += delta;
				last2ndDelta = lastDelta;
				lastDelta = delta;

				numDeltas++;
			}
#ifdef DEBUG_PS
			else {
				pulseSensor[sensorNumber].numDuplicateValues++;
			}
#endif
		}

		// When we're at the minimum samples, take a break to re-evaluate our amount of samples from it's initial value of the minimum + 1.
		if (i == pulseSensor[sensorNumber].minSamples && pulseSensor[sensorNumber].useVariableSpeedSampling) {
			// Need a temp total so it doesn't mess up the regular loops bad value rejection
			U32 tempDeltaTotal = deltaTotal;
			// Check current end value so all deltas are protected from dropped values
			if (delta > last2ndDelta * 1.8)
			{
				tempDeltaTotal -= delta;
				tempDeltaTotal += last2ndDelta;
#ifdef DEBUG_PS
				pulseSensor[sensorNumber].numDroppedValues++;
#endif
			}

			// If we're somehow left with no deltas don't divide by 0
			if (numDeltas != 0) {
				float minSamplesAverageDelta = tempDeltaTotal / (float)numDeltas; // Don't lose accuracy with int devision
				float tempFrequencyCalc = convert_delta_time_to_frequency(minSamplesAverageDelta, pulseSensor[sensorNumber].timerPeriodSeconds);

				if (tempFrequencyCalc < pulseSensor[sensorNumber].lowPulsesPerSecond) {
					numSamples = pulseSensor[sensorNumber].minSamples;
				} else if (tempFrequencyCalc > pulseSensor[sensorNumber].highPulsesPerSecond) {
					numSamples = pulseSensor[sensorNumber].maxSamples;
				} else {
					// Equation for getting how many samples we should average, which is linear from low to high samples - y = m(x-x0) + y0
					numSamples = pulseSensor[sensorNumber].vssSlope * (tempFrequencyCalc - pulseSensor[sensorNumber].lowPulsesPerSecond) + pulseSensor[sensorNumber].minSamples; // Point slope form lol
				}
			} else {
				numSamples = pulseSensor[sensorNumber].minSamples;
			}
		}

		i++;
	}

	// After the delta checking and adding loop is over, make sure the last one isn't bad either
	if (delta > lastDelta * 1.8 && delta > last2ndDelta * 1.8)
	{
		deltaTotal -= delta;
		deltaTotal += last2ndDelta;
#ifdef DEBUG_PS
		pulseSensor[sensorNumber].numDroppedValues++;
#endif
	}
	// Time to calculate results
	if(numDeltas == 0) {
		// No deltas, don't divide by 0
		*pulseSensor[sensorNumber].resultStoreLocation = 0;
		return NO_DETLAS;
	}

	// Calculate average
	float resultingAverageDelta = deltaTotal / (float)numDeltas;

	// Calculate result from average delta
	float result_frequency = convert_delta_time_to_frequency(resultingAverageDelta, pulseSensor[sensorNumber].timerPeriodSeconds);
	float result = result_frequency * pulseSensor[sensorNumber].conversionRatio;

	// Check if the calculation is inf or nan for any reason
	if(isinf(result) || isnan(result)) {
		*pulseSensor[sensorNumber].resultStoreLocation = 0;
		return INF_OR_NAN_RESULT;
	}

	if (*pulseSensor[sensorNumber].frequencyStoreLocation != 0) {
		*pulseSensor[sensorNumber].frequencyStoreLocation = result_frequency;
	}

	// Send result to store location
	*pulseSensor[sensorNumber].resultStoreLocation = result;

#ifdef DEBUG_PS
	pulseSensor[sensorNumber].averageDeltaTimerTicks = resultingAverageDelta;
	pulseSensor[sensorNumber].numSamples = numSamples;
	pulseSensor[sensorNumber].DMACurrentPosition = DMACurrentPosition;
#endif

	return READ_SUCCESS;
}

// Function exactly as name implies
static void clear_buffer_and_reset_dma(U8 sensorNumber) {
	HAL_TIM_IC_Stop_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel);
	memset(pulseSensor[sensorNumber].buffer, 0, sizeof(U32)*IC_BUF_SIZE);	// Use memset function to set all memory in buffer to 0 with the byte size of 64 U32 values.
	HAL_TIM_IC_Start_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel, (U32*)(pulseSensor[sensorNumber].buffer), IC_BUF_SIZE);
}

// Also as name implies
static float convert_delta_time_to_frequency(float deltaTime, float timerPeriodSeconds) {
	float lengthOfDeltaSeconds = deltaTime * timerPeriodSeconds; // Since timer period is division already done (1MHz is 1000ns period)

	return 1 / lengthOfDeltaSeconds;
}


