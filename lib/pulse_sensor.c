/**
 * Library for operating pulse sensors
 *
 * Alex Tong
 *
 * Testing list -
 * - Timer size setup
 * - RPM conversion works correctly
 * - Variable sampling works correctly
 */

// TODO: Decrease jittering

#include "stm32f4xx_hal.h"
#include "pulse_sensor.h"
#include "base_types.h"
#include <stdbool.h>
#include <stdio.h>

//static U16 buffer[IC_BUF_SIZE] = {0};
PulseSensor pulseSensor[TIMER_COUNT] = {0};

static U16 averageSpeedTimerTicks = 0;
static U16 averageSpeedMsPerTick = 0;

// Debug Variables Start
static S16 TrackedDMACurrentPosition = 0;
static U32 TrackedMostRecentDelta = 0;
static U16 TrackedAmountOfSamples = 0;
static U32 TrackedValueInQuestion = 0;
static U32 TrackedFirstValue = 0;
static U32 TrackedLastValue = 0;
static U16 TrackedRPM = 0;
static U32 TrackedDeltaList[IC_BUF_SIZE] = {0};
static U32 TrackedBufferCopy[IC_BUF_SIZE] = {0};
static U32 TrackedDeltaTotal = 0;
static U16 TrackedSignalRate = 0;
static float TrackedResult = 0;
static U32 lastTick = 0;
static bool error = false;
// Debug Variables End

int numSensors = 0;

static void check_timer_dma(int sensorNumber);
static U16 convert_delta_time_to_rpm(U16 value, float conversionRatio);
static void clear_buffer_and_reset_dma();

void setup_timer_and_start_dma_vss(
		TIM_HandleTypeDef* htim,
		U32 channel,
		U32 timerPeriodNs,
		U16 conversionRatio,
		float* resultStoreLocation,
		bool useVariableSpeedSampling,
		U16 lowResultingValue,
		U16 highResultingValue,
		U16 minSamples
		)
{
	int timerCount = numSensors;

	// Set the local values of the pulse sensor data struct to the given parameters
	pulseSensor[timerCount].htim = htim;
	pulseSensor[timerCount].channel = channel;
	pulseSensor[timerCount].timerPeriodNs = timerPeriodNs; // Period of timer ticks (they can be different between different timers)
	pulseSensor[timerCount].conversionRatio = conversionRatio; // ticks per rev
	pulseSensor[timerCount].resultStoreLocation = resultStoreLocation;
	pulseSensor[timerCount].useVariableSpeedSampling = useVariableSpeedSampling;
	pulseSensor[timerCount].lowResultingValue = lowResultingValue; // set to desired number of samples if not using variable speed sampling, or 0 for max samples
	pulseSensor[timerCount].highResultingValue = highResultingValue; // set to 0 if not using variable speed sampling
	pulseSensor[timerCount].minSamples = minSamples;

	// TODO: Check the size of the timer
	if (htim->Instance->ARR == 0xFFFF) {
		pulseSensor[timerCount].timerSize = 16;
	} else {
		pulseSensor[timerCount].timerSize = 32;
	}
	pulseSensor[timerCount].timerSize = 16;

	// Clear the sensor's buffer
	for(int i = 0; i < IC_BUF_SIZE; i++) {
		pulseSensor[timerCount].buffer[i] = 0;
	}

	pulseSensor[timerCount].averageSpeedTimerTicks = 0;
	pulseSensor[timerCount].lastDMAReadValueTimeMs = 0;
	pulseSensor[timerCount].DMA_lastReadValue = 0;
	pulseSensor[timerCount].stopped = true;

	HAL_TIM_IC_Start_DMA(htim, channel, (U32*)pulseSensor[numSensors].buffer, IC_BUF_SIZE);

	numSensors++; // Track how many sensors have been set up
}

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
		0,
		0,
		0 // Min samples
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
static void check_timer_dma(int sensorNumber) {
	// Copy all the values so they can't change while doing calculations, starting at the DMA position because we know stopping the buffer resets its position.
	U32 bufferCopy[IC_BUF_SIZE] = {0};
	HAL_TIM_IC_Stop_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel);
	// Find the current position of DMA for the current sensor  - Note: Important this happens after DMA stops or weird values will randomly occur
	S16 DMACurrentPosition = IC_BUF_SIZE - (U16)((pulseSensor[sensorNumber].htim->hdma[1])->Instance->NDTR);

	for (U16 c = 0; c < IC_BUF_SIZE; c++)
	{
		S16 i = (DMACurrentPosition + c) % IC_BUF_SIZE;
		bufferCopy[c] = pulseSensor[sensorNumber].buffer[i];
	}
	// Copy values of buffer copy into original buffer so its values start at 0
	for (U16 i = 0; i < IC_BUF_SIZE; i++) {
		pulseSensor[sensorNumber].buffer[i] = bufferCopy[i];
	}
	HAL_TIM_IC_Start_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel, (U32*)(pulseSensor[sensorNumber].buffer), IC_BUF_SIZE); // Note: Resets DMA position

	// Find the value in question, which is 1 position backwards from the DMA's current position, which is the end of the buffer copy.
	U32 valueInQuestion = bufferCopy[IC_BUF_SIZE - 1];

	if (pulseSensor[sensorNumber].stopped) {	// If we already know we're stopped potentially end early.
		if (valueInQuestion != 0) {	// If we were previously stopped but may be moving again.
			// Check if this is just really slow occasional
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
			*(pulseSensor[sensorNumber].resultStoreLocation) = 0; // Otherwise make sure the store location vlaue is 0 and leave.
			return;
		}
	}

	if (pulseSensor[sensorNumber].DMA_lastReadValue == valueInQuestion) {	// Check if the last read value is the same as the current
		if (HAL_GetTick() - pulseSensor[sensorNumber].lastDMAReadValueTimeMs >= DMA_STOPPED_TIMEOUT_MS){	// Check if we haven't changed values in a while which might mean we're stopped
			pulseSensor[sensorNumber].stopped = true;

			// Clear buffer so any non-zero values will be quickly identified that the car is moving again. This will also reset the DMA position.
			clear_buffer_and_reset_dma(sensorNumber);

			// TODO: VERIFY
			*(pulseSensor[sensorNumber].resultStoreLocation) = 0;
			return;
		}
		*(pulseSensor[sensorNumber].resultStoreLocation) = pulseSensor[sensorNumber].averageSpeedTimerTicks;
		return;
	}

	// --- Passed All Breakpoint Checks ---
	pulseSensor[sensorNumber].DMA_lastReadValue = valueInQuestion;

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
	U16 tempRPMCalc = convert_delta_time_to_rpm(mostRecentDelta, pulseSensor[sensorNumber].conversionRatio);
	if(pulseSensor[sensorNumber].useVariableSpeedSampling) {
		if (tempRPMCalc < pulseSensor[sensorNumber].lowResultingValue) {
			amountOfSamples = pulseSensor[sensorNumber].minSamples;
		} else if (tempRPMCalc > pulseSensor[sensorNumber].highResultingValue) {
			amountOfSamples = IC_BUF_SIZE;
		} else {
			// Equation for getting how many samples we should average, which is linear from low to high samples
			float slope = (IC_BUF_SIZE - pulseSensor[sensorNumber].minSamples) /
					(float)(pulseSensor[sensorNumber].highResultingValue - pulseSensor[sensorNumber].lowResultingValue);
			amountOfSamples = slope * (tempRPMCalc - pulseSensor[sensorNumber].lowResultingValue) + pulseSensor[sensorNumber].minSamples; // Point slope form lol
		}
	} else {
		amountOfSamples = pulseSensor[sensorNumber].highResultingValue;
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

	// Debug Start
	TrackedDMACurrentPosition = DMACurrentPosition;
	TrackedMostRecentDelta = mostRecentDelta;
	TrackedAmountOfSamples = amountOfSamples;
	TrackedValueInQuestion = valueInQuestion;
	TrackedFirstValue = bufferCopy[IC_BUF_SIZE - 1];
	TrackedLastValue = bufferCopy[0];
	TrackedRPM = 0;
	for(int i = 0; i < IC_BUF_SIZE; i++) {
		TrackedDeltaList[i] = deltaList[i];
		TrackedBufferCopy[i] = bufferCopy[i];
	}
	TrackedDeltaTotal = deltaTotal;
	TrackedSignalRate = 0;
	// Debug end

	pulseSensor[sensorNumber].lastDMAReadValueTimeMs = HAL_GetTick();

	float result = deltaTotal / numDeltas;

	if (error) {
		error = false;
	} else {
		if(result >= TrackedResult * 1.3 || result <= TrackedResult * 0.7) {
			printf("--- Anomaly Detected! ---\n");
			printf("Current Value: %f\n", result);
			printf("Previous Value: %f\n", TrackedResult);
			printf("DMA Position: %u\n", TrackedDMACurrentPosition);
			printf("Amount of Samples: %u\n", TrackedAmountOfSamples);
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastTick);
			lastTick = HAL_GetTick();
			for (int i = 0; i < IC_BUF_SIZE; i++) {
				printf("Value %i: ", i);
				if (i == DMACurrentPosition) {
					printf("- ");
				}
				printf("%lu\n", bufferCopy[i]);
			}
			printf("DELTAS ===\n");
			for (int i = 0; i < IC_BUF_SIZE; i++) {
				printf("Value %i: ", i);
				printf("%u\n", deltaList[i]);
				//(U16)((DMACurrentPosition - i + IC_BUF_SIZE) % IC_BUF_SIZE)
			}
			error = true;
		}
	}

	TrackedResult = result;
	pulseSensor[sensorNumber].averageSpeedTimerTicks = result;
	*pulseSensor[sensorNumber].resultStoreLocation = result; //TODO: Check

	TrackedRPM = convert_delta_time_to_rpm(result, pulseSensor[sensorNumber].conversionRatio);
}

static void clear_buffer_and_reset_dma(int sensorNumber) {
	HAL_TIM_IC_Stop_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel);
	for (U16 c = 0; c < IC_BUF_SIZE - 1; c++)
	{
		pulseSensor[sensorNumber].buffer[c] = 0;
	}
	HAL_TIM_IC_Start_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel, (U32*)(pulseSensor[sensorNumber].buffer), IC_BUF_SIZE);
}


// Debug
//static float TrackedValue = 0;
//static double TrackedLengthOfDeltaSeconds = 0;
//static double TrackedSecondsFor1Revolution = 0;
// Debug End

static U16 convert_delta_time_to_rpm(U16 value, float conversionRatio) {
	double lengthOfDeltaSeconds = (float)value / ONE_MHZ;
	double secondsFor1Revolution = lengthOfDeltaSeconds * conversionRatio;
	U16 rpm = (U16)(60 / secondsFor1Revolution); // 60 is to get from seconds to minutes

	// Debug
//	TrackedValue = value;
//	TrackedLengthOfDeltaSeconds = lengthOfDeltaSeconds;
//	TrackedSecondsFor1Revolution = secondsFor1Revolution;
	//Debug End

	return rpm;
}
