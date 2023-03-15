/**
 * Library for operating pulse one or multiple sensors and returning an RPM through a float pointer.
 *
 * Alex Tong
 */

// TODO: Decrease jittering

#include "stm32f4xx_hal.h"
#include "pulse_sensor.h"
#include "base_types.h"
#include <stdbool.h>
#include <stdio.h>

PulseSensor pulseSensor[TIMER_COUNT] = {0};
int numSensors = 0;

static void clear_buffer_and_reset_dma();
static U16 convert_delta_time_to_rpm(U16 deltaTime, float conversionRatio);

// Function for setting up timer with all details, use this function directly to include variable speed sampling (vss) data.
void setup_timer_and_start_dma_vss(
		TIM_HandleTypeDef* htim,
		U32 channel,
		U32 timerPeriodNs,
		float conversionRatio,
		float* resultStoreLocation,
		bool useVariableSpeedSampling,
		U16 lowResultingRPMValue,
		U16 highResultingRPMValue,
		U16 minSamples
		)
{
	int timerCount = numSensors;

	// Set the local values of the pulse sensor data struct to the given parameters
	pulseSensor[timerCount].htim = htim; // Timer that was used to set up input capture and DMA on
	pulseSensor[timerCount].channel = channel; // Channel of the given timer
	pulseSensor[timerCount].timerPeriodNs = timerPeriodNs; // Period of timer ticks (they can be different between different timers)
	pulseSensor[timerCount].conversionRatio = conversionRatio; // ticks per rev
	pulseSensor[timerCount].resultStoreLocation = resultStoreLocation; // Float pointer to a location you want the resulting RPM to update
	pulseSensor[timerCount].useVariableSpeedSampling = useVariableSpeedSampling; // Bool for weather or not to vary the amount of samples taken from the buffer to determine the resulting RPM average
	pulseSensor[timerCount].lowResultingRPMValue = lowResultingRPMValue; // RPM at which the min samples will be used from the buffer behind the DMA position. Set to desired number of samples if not using variable speed sampling, or 0 for max samples
	pulseSensor[timerCount].highResultingRPMValue = highResultingRPMValue; // RPM value at which all 64 values in the buffer are sampled to get the resulting speed value. Set to 0 if not using variable speed sampling
	pulseSensor[timerCount].minSamples = minSamples;	// Minimum amount of samples to take if using variable speed sampling

	// Evaluate the size of the given timer by checking if the Auto reload value is the max size of a 16bit value 0xFFFF
	// TODO: Verify the 32 bit detection works (16 bit validated, so it should work)
	if (htim->Instance->ARR == 0xFFFF) {
		pulseSensor[timerCount].timerSize = 16;
	} else {
		pulseSensor[timerCount].timerSize = 32;
	}

	// Clear the sensor's buffer
	for(int i = 0; i < IC_BUF_SIZE; i++) {
		pulseSensor[timerCount].buffer[i] = 0;
	}

	// Default other values to 0
	pulseSensor[timerCount].averageDeltaTimerTicks = 0;
	pulseSensor[timerCount].lastDMAReadValueTimeMs = 0;
	pulseSensor[timerCount].DMA_lastReadValue = 0;
	pulseSensor[timerCount].stopped = true;

	HAL_TIM_IC_Start_DMA(htim, channel, (U32*)pulseSensor[numSensors].buffer, IC_BUF_SIZE);

	numSensors++; // Track how many sensors have been set up
}

// Function for setting up timer without variable speed sampling (vss)
void setup_timer_and_start_dma(
		TIM_HandleTypeDef* htim,
		U32 channel,
		U32 timerPeriodNs,
		float conversionRatio,
		float* resultStoreLocation
		)
{
	setup_timer_and_start_dma_vss(
		htim,
		channel,
		timerPeriodNs,
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
	int activeTimerCount = numSensors;

	for (int sensorNumber = 0; sensorNumber < activeTimerCount; sensorNumber++) {
		check_timer_dma(sensorNumber);
	}
}

// Function that goes through the buffer of the given pulse sensor, handling edge cases, and setting the return value to the found speed value.
void check_timer_dma(int sensorNumber) {

	U32 bufferCopy[IC_BUF_SIZE] = {0};

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
	for (U16 i = 0; i < IC_BUF_SIZE; i++) {
		pulseSensor[sensorNumber].buffer[i] = bufferCopy[i];
	}

	// Restart DMA before doing calculations so we lose as little values as possible.
	HAL_TIM_IC_Start_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel, (U32*)(pulseSensor[sensorNumber].buffer), IC_BUF_SIZE); // Note: Resets DMA position

	// Find the value in question, which is 1 position backwards from the DMA's current position, which is the end of the buffer copy.
	U32 valueInQuestion = bufferCopy[IC_BUF_SIZE - 1];

	if (pulseSensor[sensorNumber].stopped) {	// If we already know we're stopped, check if we should end early
		if (valueInQuestion != 0) {	// If we were previously stopped but may be moving again.
			// Check if this is just really a slow and occasional value
			if (valueInQuestion == pulseSensor[sensorNumber].DMA_lastReadValue) {
				// Clear the random values that aren't fast enough so they don't sit and become stale
				clear_buffer_and_reset_dma(sensorNumber);
				return;
			} else if (HAL_GetTick() - pulseSensor[sensorNumber].lastDMAReadValueTimeMs <= DMA_STOPPED_TIMEOUT_MS) {
				pulseSensor[sensorNumber].stopped = false; // Declare we are no longer stopped and move on.
			} else {
				pulseSensor[sensorNumber].DMA_lastReadValue = valueInQuestion;
				pulseSensor[sensorNumber].lastDMAReadValueTimeMs = HAL_GetTick();
				return;
			}
		} else {
			*(pulseSensor[sensorNumber].resultStoreLocation) = 0; // Otherwise make sure the store location value is 0 and leave.
			return;
		}
	}

	if (pulseSensor[sensorNumber].DMA_lastReadValue == valueInQuestion) {	// Check if the last read value is the same as the current
		if (HAL_GetTick() - pulseSensor[sensorNumber].lastDMAReadValueTimeMs >= DMA_STOPPED_TIMEOUT_MS){	// Check if we haven't changed values in a while which might mean we're stopped
			pulseSensor[sensorNumber].stopped = true;

			// Clear buffer so any non-zero values will be quickly identified that the car is moving again. This will also reset the DMA position.
			clear_buffer_and_reset_dma(sensorNumber);

			*(pulseSensor[sensorNumber].resultStoreLocation) = 0;
			return;
		}
		*(pulseSensor[sensorNumber].resultStoreLocation) = pulseSensor[sensorNumber].averageDeltaTimerTicks;
		return;
	}

	// === Passed All Breakpoint Checks ===
	pulseSensor[sensorNumber].DMA_lastReadValue = valueInQuestion;
	pulseSensor[sensorNumber].lastDMAReadValueTimeMs = HAL_GetTick();

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
		// Get an RPM calc from the most recent delta so the user can input RPM values as high and low values rather than deltas
		U16 tempRPMCalc = convert_delta_time_to_rpm(mostRecentDelta, pulseSensor[sensorNumber].conversionRatio);

		if (tempRPMCalc < pulseSensor[sensorNumber].lowResultingRPMValue) {
			amountOfSamples = pulseSensor[sensorNumber].minSamples;
		} else if (tempRPMCalc > pulseSensor[sensorNumber].highResultingRPMValue) {
			amountOfSamples = IC_BUF_SIZE;
		} else {
			// Equation for getting how many samples we should average, which is linear from low to high samples
			float slope = (IC_BUF_SIZE - pulseSensor[sensorNumber].minSamples) /
					(float)(pulseSensor[sensorNumber].highResultingRPMValue - pulseSensor[sensorNumber].lowResultingRPMValue);
			amountOfSamples = slope * (tempRPMCalc - pulseSensor[sensorNumber].lowResultingRPMValue) + pulseSensor[sensorNumber].minSamples; // Point slope form lol
		}
	} else {
		amountOfSamples = pulseSensor[sensorNumber].highResultingRPMValue;
		if (amountOfSamples == 0) {
			amountOfSamples = IC_BUF_SIZE;
		}
	}

	// Calculate the deltas between each of the time values and store them in deltaList
	U16 deltaList[IC_BUF_SIZE] = {0};
	U16 numDeltas = 0;
	for (U16 c = 0; c < amountOfSamples - 1; c++)
	{
		S16 i = (IC_BUF_SIZE - 1 - c);
		U16 value1 = bufferCopy[i];
		U16 value2 = bufferCopy[i - 1];
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
	U32 deltaTotal = 0;
	for (U16 c = 0; c < numDeltas; c++) {
		deltaTotal += deltaList[c];
	}

	// Calculate average
	float resultingAverageDelta = deltaTotal / numDeltas;

	TrackedResult = resultingAverageDelta;
	pulseSensor[sensorNumber].averageDeltaTimerTicks = resultingAverageDelta;

	// Calculate RPM from average delta
	U16 RPM = convert_delta_time_to_rpm(resultingAverageDelta, pulseSensor[sensorNumber].conversionRatio);

	// Send result to store location
	*pulseSensor[sensorNumber].resultStoreLocation = RPM;
}

// Function exactly as name implies
static void clear_buffer_and_reset_dma(int sensorNumber) {
	HAL_TIM_IC_Stop_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel);
	for (U16 c = 0; c < IC_BUF_SIZE - 1; c++)
	{
		pulseSensor[sensorNumber].buffer[c] = 0;
	}
	HAL_TIM_IC_Start_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel, (U32*)(pulseSensor[sensorNumber].buffer), IC_BUF_SIZE);
}

// Also as name implies
static U16 convert_delta_time_to_rpm(U16 deltaTime, float conversionRatio) {
	// TODO: Implement timer period in here once I figure out how that factors in
	double lengthOfDeltaSeconds = (float)deltaTime / ONE_MHZ;
	double secondsFor1Revolution = lengthOfDeltaSeconds * conversionRatio;
	float rpm = (60 / secondsFor1Revolution); // 60 is to get from seconds to minutes
	return rpm;
}
