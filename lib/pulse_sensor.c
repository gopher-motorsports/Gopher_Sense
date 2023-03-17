/**
 * Library for operating pulse one or multiple sensors and returning an result through a float pointer.
 *
 * Alex Tong
 */

// TODO: Decrease jittering

#include "stm32f4xx_hal.h"
#include "pulse_sensor.h"
#include "base_types.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

PulseSensor pulseSensor[TIMER_COUNT] = {0};
U8 numSensors = 0;

static void clear_buffer_and_reset_dma(U8 sensorNumber);
static U16 convert_delta_time_to_frequency(U32 deltaTime, float timerPeriodSeconds);

// Function for setting up timer with all details, use this function directly to include variable speed sampling (vss) data.
void setup_timer_and_start_dma_vss(
		TIM_HandleTypeDef* htim,
		U32 channel,
		float conversionRatio,
		float* resultStoreLocation,
		bool useVariableSpeedSampling,
		U16 lowPulsesPerSecond,
		U16 highPulsesPerSecond,
		U16 minSamples
		)
{
	PulseSensor* newSensor = &pulseSensor[numSensors];

	// Set the local values of the pulse sensor data struct to the given parameters
	newSensor->htim = htim;
	newSensor->channel = channel;
	newSensor->conversionRatio = conversionRatio;
	newSensor->resultStoreLocation = resultStoreLocation;
	newSensor->useVariableSpeedSampling = useVariableSpeedSampling;
	newSensor->lowPulsesPerSecond = lowPulsesPerSecond;
	newSensor->highPulsesPerSecond = highPulsesPerSecond;
	newSensor->minSamples = minSamples;

	U32 clockFrequency = HAL_RCC_GetSysClockFreq();

	// Evaluate the size of the given timer by checking if the Auto reload value is the max size of a 16bit value 0xFFFF
	// TODO: Verify the 32 bit detection works (16 bit validated, so it should work)
	if (htim->Instance->ARR == 0xFFFF) {
		newSensor->timerSize = 16;
		clockFrequency *= 0.5;
	} else {
		newSensor->timerSize = 32;
	}

	newSensor->timerPeriodSeconds = 1 / ((float)clockFrequency / (htim->Instance->PSC + 1));

	// Clear the sensor's buffer
	memset(newSensor->buffer, 0, sizeof(U32)*IC_BUF_SIZE);

	// Default other values to 0
	newSensor->averageDeltaTimerTicks = 0;
	newSensor->lastDMAReadValueTimeMs = 0;
	newSensor->DMA_lastReadValue = 0;
	newSensor->stopped = true;

	HAL_TIM_IC_Start_DMA(htim, channel, (U32*)pulseSensor[numSensors].buffer, IC_BUF_SIZE);
}

// Function for setting up timer without variable speed sampling (vss)
void setup_timer_and_start_dma(
		TIM_HandleTypeDef* htim,
		U32 channel,
		float conversionRatio,
		float* resultStoreLocation
		)
{
	setup_timer_and_start_dma_vss(
		htim,
		channel,
		conversionRatio,
		resultStoreLocation,
		false,
		0,	// Low RPM Value
		0,	// High RPM Value
		0 	// Min samples
		);
}

// Function for going through all of the currently set up pulse sensors
void check_all_dmas() {
	for (int sensorNumber = 0; sensorNumber < numSensors; sensorNumber++) {
		check_timer_dma(sensorNumber);
	}
}

// Function that goes through the buffer of the given pulse sensor, handling edge cases, and setting the return value to the found speed value.
void check_timer_dma(int sensorNumber) {
	U32 bufferCopy[IC_BUF_SIZE] = {0};

	// Get the current tick and use it throughout the whole function so it can't change between operations.
	U32 currentTick = HAL_GetTick();

	// Stop DMA so we know that values won't change when copying them to bufferCopy
	HAL_TIM_IC_Stop_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel);

	// Find the current position of DMA for the current sensor  - Note: Important this happens after DMA stops or weird values will randomly occur
	S16 DMACurrentPosition = IC_BUF_SIZE - (U16)((pulseSensor[sensorNumber].htim->hdma[1])->Instance->NDTR);

	// Copy all the values to bufferCopy so they can't change while doing calculations,
	// starting at the DMA position because we know stopping the buffer resets its position.
	for (U16 c = 0; c < IC_BUF_SIZE; c++)
	{
		S16 i = (DMACurrentPosition + c) % IC_BUF_SIZE;
		bufferCopy[c] = pulseSensor[sensorNumber].buffer[i];
	}

	// Copy values of buffer copy into original buffer so its oldest values start at position 0,
	// which is where the DMA position will be placed after restarting.
	memcpy(pulseSensor[sensorNumber].buffer, bufferCopy, sizeof(U32)*IC_BUF_SIZE);



	// Restart DMA before doing calculations so we lose as little values as possible.
	HAL_TIM_IC_Start_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel, (U32*)(pulseSensor[sensorNumber].buffer), IC_BUF_SIZE); // Note: Resets DMA position

	// Find the value in question, which is 1 position backwards from the DMA's current position, which is the end of the buffer copy.
	U32 valueInQuestion = bufferCopy[IC_BUF_SIZE - 1];

	if (pulseSensor[sensorNumber].stopped) {	// If we already know we're stopped, check if we should end early
		if (valueInQuestion != 0) {	// If we were previously stopped but may be moving again.
			// Check if this is just really a slow and occasional value
			if (valueInQuestion == pulseSensor[sensorNumber].DMA_lastReadValue) {
				// We're still seeing the same value, is it old now? We can't get here if it was last a 0.
				if (currentTick - pulseSensor[sensorNumber].lastDMAReadValueTimeMs >= DMA_STOPPED_TIMEOUT_MS) {
					// Clear the random values that aren't fast enough so they don't sit and become stale
					clear_buffer_and_reset_dma(sensorNumber);
					pulseSensor[sensorNumber].DMA_lastReadValue = 0; // Set to 0 so there aren't accidents.
				}
				return;
			} else {
				if (pulseSensor[sensorNumber].DMA_lastReadValue != 0) {
					// The value is new, it didn't get wiped, and the last wasn't 0 so we're good again
					pulseSensor[sensorNumber].stopped = false; // Declare we are no longer stopped and move on.
				} else {
					// A new unwiped value but it's just the first after 0. Log it and go again.
					pulseSensor[sensorNumber].DMA_lastReadValue = valueInQuestion;
					pulseSensor[sensorNumber].lastDMAReadValueTimeMs = currentTick;
					return;
				}
			}
		} else {
			*(pulseSensor[sensorNumber].resultStoreLocation) = 0; // Otherwise make sure the store location value is 0 and leave.
			pulseSensor[sensorNumber].lastDMAReadValueTimeMs = currentTick;
			return;
		}
	}

	if (pulseSensor[sensorNumber].DMA_lastReadValue == valueInQuestion) {	// Check if the last read value is the same as the current
		if (currentTick - pulseSensor[sensorNumber].lastDMAReadValueTimeMs >= DMA_STOPPED_TIMEOUT_MS){	// Check if we haven't changed values in a while which might mean we're stopped
			pulseSensor[sensorNumber].stopped = true;

			// Clear buffer so any non-zero values will be quickly identified that the car is moving again. This will also reset the DMA position.
			clear_buffer_and_reset_dma(sensorNumber);

			pulseSensor[sensorNumber].DMA_lastReadValue = 0;
			pulseSensor[sensorNumber].lastDMAReadValueTimeMs = currentTick;
			*(pulseSensor[sensorNumber].resultStoreLocation) = 0;
			return;
		}
		// Value is the same as the last but it hasn't been enough time to declare stale, hop out of here for now because there's nothing new to process
		return;
	}

	// === Passed All Breakpoint Checks ===
	pulseSensor[sensorNumber].DMA_lastReadValue = valueInQuestion;
	pulseSensor[sensorNumber].lastDMAReadValueTimeMs = currentTick;

	// Calculate the amount of samples to take to get the speed based on the delta between the last 2 values
	U32 value2 = bufferCopy[IC_BUF_SIZE - 2];
	U32 mostRecentDelta = 0;
	// Find the the most rececnt difference between 2 values, with roll-over protection
	if (valueInQuestion < value2) {
		mostRecentDelta = ((1 << pulseSensor[sensorNumber].timerSize) | valueInQuestion) - value2; // Buffer rollover protection
	} else {
		mostRecentDelta = valueInQuestion - value2;
	}

	U16 amountOfSamples = 0;

	if(pulseSensor[sensorNumber].useVariableSpeedSampling) {
		// Get an frequency calc from the most recent delta so the user can input RPM values as high and low values rather than deltas
		U16 tempFrequencyCalc = convert_delta_time_to_frequency(mostRecentDelta, pulseSensor[sensorNumber].timerPeriodSeconds);

		if (tempFrequencyCalc < pulseSensor[sensorNumber].lowPulsesPerSecond) {
			amountOfSamples = pulseSensor[sensorNumber].minSamples;
		} else if (tempFrequencyCalc > pulseSensor[sensorNumber].highPulsesPerSecond) {
			amountOfSamples = IC_BUF_SIZE;
		} else {
			// Equation for getting how many samples we should average, which is linear from low to high samples
			float slope = (IC_BUF_SIZE - pulseSensor[sensorNumber].minSamples) /
					(float)(pulseSensor[sensorNumber].highPulsesPerSecond - pulseSensor[sensorNumber].lowPulsesPerSecond);
			amountOfSamples = slope * (tempFrequencyCalc - pulseSensor[sensorNumber].lowPulsesPerSecond) + pulseSensor[sensorNumber].minSamples; // Point slope form lol
		}
	} else {
		amountOfSamples = IC_BUF_SIZE;
	}

	// Calculate the deltas between each of the time values and store them in deltaList
	U32 deltaList[IC_BUF_SIZE] = {0};
	U16 numDeltas = 0;
	for (U16 c = 0; c < amountOfSamples - 1; c++)
	{
		S16 i = (IC_BUF_SIZE - 1 - c);
		U32 value1 = bufferCopy[i];
		U32 value2 = bufferCopy[i - 1];
		if (value1 != 0 && value2 != 0) { // Check if 0s because of previously empty buffer, because otherwise a value would simply be a time amount
			if (value1 < value2) {
				deltaList[numDeltas] = ((1 << pulseSensor[sensorNumber].timerSize) | value1) - value2; // Roll-over (resets to 0) protection
			} else {
				deltaList[numDeltas] = value1 - value2;
			}
			numDeltas++;
		}
	}

	// Get average of deltas
	U64 deltaTotal = 0;
	for (U16 c = 0; c < numDeltas; c++) {
		deltaTotal += deltaList[c];
	}

	// Calculate average
	float resultingAverageDelta = deltaTotal / numDeltas;
	pulseSensor[sensorNumber].averageDeltaTimerTicks = resultingAverageDelta;

	// Calculate result from average delta
	U16 result = convert_delta_time_to_frequency(resultingAverageDelta, pulseSensor[sensorNumber].timerPeriodSeconds) * pulseSensor[sensorNumber].conversionRatio;

	// Send result to store location
	*pulseSensor[sensorNumber].resultStoreLocation = result;
}

// Function exactly as name implies
static void clear_buffer_and_reset_dma(U8 sensorNumber) {
	HAL_TIM_IC_Stop_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel);
	memset(pulseSensor[sensorNumber].buffer, 0, sizeof(U32)*IC_BUF_SIZE);	// Use memset function to set all memory in buffer to 0 with the byte size of 64 U32 values.
	HAL_TIM_IC_Start_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel, (U32*)(pulseSensor[sensorNumber].buffer), IC_BUF_SIZE);
}

// Also as name implies
static U16 convert_delta_time_to_frequency(U32 deltaTime, float timerPeriodSeconds) {
	float lengthOfDeltaSeconds = deltaTime * timerPeriodSeconds; // Since timer period is division already done (1MHz is 1000ns period)
	U16 frequencyHz = 1 / lengthOfDeltaSeconds;
	return frequencyHz;
}
