/**
 * Library for operating pulse sensors
 *
 * Alex Tong
 */

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
static U16 TrackedMostRecentDelta = 0;
static U16 TrackedAmountOfSamples = 0;
static U16 TrackedValueInQuestion = 0;
static U16 TrackedFirstValue = 0;
static U16 TrackedLastValue = 0;
static U16 TrackedRPM = 0;
static U16 TrackedDeltaList[IC_BUF_SIZE] = {0};
static U16 TrackedBufferCopy[IC_BUF_SIZE] = {0};
static U32 TrackedDeltaTotal = 0;
static U16 TrackedSignalRate = 0;
static float TrackedResult = 0;
static U32 lastTick = 0;
static S16 DMACurrentPosition2;
static U32 TIMEVAL = 0;

// Debug Variables End

int numSensors = 0;

static void checkDMA(int sensorNumber);

//void test8888(void) {
//
//	S16 DMACurrentPosition = IC_BUF_SIZE - (U16)((pulseSensor[0].htim->hdma[1])->Instance->NDTR);
//	U16 bufferCopy[IC_BUF_SIZE] = {0};
//	for (U16 c = 0; c < IC_BUF_SIZE; c++)
//	{
//		bufferCopy[c] = pulseSensor[0].buffer[c];
//	}
//
//	if(HAL_GetTick() - TIMEVAL > 1000) {
//		printf("*** Periodic ***\n");
//		printf("DMA Current Position %i\n", DMACurrentPosition);
//		//printf("Current Value: %f\n", result);
////		printf("Previous Value: %f\n", TrackedResult);
////		printf("DMA Position: %u\n", TrackedDMACurrentPosition);
////		printf("Amount of Samples: %u\n", TrackedAmountOfSamples);
//		printf("Current Tick: %lu\n", HAL_GetTick());
//		printf("Distance from Last Occurence: %ul\n", HAL_GetTick() - lastTick);
//		lastTick = HAL_GetTick();
//		for (int i = 0; i < IC_BUF_SIZE; i++) {
//			printf("Value %i: ", i);
//			if (i == DMACurrentPosition) {
//				printf("- ");
//			}
//			printf("%u\n", bufferCopy[i]);
//		}
////		printf("DELTAS ===\n");
////		for (int i = 0; i < IC_BUF_SIZE; i++) {
////			printf("Value %i: ", i);
////			printf("%u\n", deltaList[i]);
////		}
//		TIMEVAL = HAL_GetTick();
//	}
//}

// TODO: Figure out why this returns strange values at low speeds and fast speed changes.

void setupTimerAndStartDMA(TIM_HandleTypeDef* htim, U32 channel, U32 timerPeriodNs, float conversionRatio, float* resultStoreLocation, bool useVariableSpeedSampling, U16 lowTimeDelta, U16 highTimeDelta) {
	int timerCount = numSensors;

	// Set the local values of the pulse sensor data struct to the given parameters
	pulseSensor[timerCount].htim = htim;
	pulseSensor[timerCount].channel = channel;
	pulseSensor[timerCount].timerPeriodNs = timerPeriodNs; // Period of timer ticks (they can be different between different timers)
	pulseSensor[timerCount].conversionRatio = conversionRatio; // ticks per rev
	pulseSensor[timerCount].resultStoreLocation = resultStoreLocation;
	pulseSensor[timerCount].useVariableSpeedSampling = useVariableSpeedSampling;
	pulseSensor[timerCount].lowTimeDeltaValue = lowTimeDelta; // set to desired number of samples if not using variable speed sampling, or 0 for max samples
	pulseSensor[timerCount].highTimeDeltaValue = highTimeDelta; // set to 0 if not using variable speed sampling

	// Clear the sensor's buffer
	for(int i = 0; i < IC_BUF_SIZE; i++) {
		pulseSensor[timerCount].buffer[i] = 0;
	}

	pulseSensor[timerCount].averageSpeedTimerTicks = 0;
	pulseSensor[timerCount].lastDMAReadValueTimeMs = 0;
	pulseSensor[timerCount].DMA_lastReadValue = 0;
	pulseSensor[timerCount].stopped = false;

	HAL_TIM_IC_Start_DMA(htim, channel, (U32*)pulseSensor[numSensors].buffer, IC_BUF_SIZE);

	numSensors++; // Track how many sensors have been set up
}

// Function for going through all of the currently set up pulse sensors
void checkAllDMAs() {
	int activeTimerCount = numSensors;

	for (int sensorNumber = 0; sensorNumber < activeTimerCount; sensorNumber++) {
		//if (HAL_GetTick() - pulseSensor[sensorNumber].lastDMACheckMs >= timerPeriodNs) {
			checkDMA(sensorNumber);
	}
}

// Function that goes through the buffer of the given pulse sensor, handling edge cases, and setting the return value to the found speed value.
static void checkDMA(int sensorNumber) {
	// Find the current position of DMA for the current sensor
	S16 DMACurrentPosition = IC_BUF_SIZE - (U16)((pulseSensor[sensorNumber].htim->hdma[1])->Instance->NDTR);
//	uint32_t test = 22;
//	pulseSensor[sensorNumber].htim->hdma[1]->Instance->NDTR = test;
//	DMACurrentPosition2 = IC_BUF_SIZE - (U16)((pulseSensor[sensorNumber].htim->hdma[1])->Instance->NDTR);


	// Copy all the values so they can't change while doing calculations, starting at the DMA position because we know stopping the buffer resets its position.
	U16 bufferCopy[IC_BUF_SIZE] = {0};
	HAL_TIM_IC_Stop_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel);
	for (U16 c = 0; c < IC_BUF_SIZE; c++)
	{
		S16 i = (DMACurrentPosition + c) % IC_BUF_SIZE;
		bufferCopy[c] = pulseSensor[sensorNumber].buffer[i];
	}
	// Copy values of buffer copy into original buffer so its values start at 0
	for (U16 i = 0; i < IC_BUF_SIZE; i++) {
		pulseSensor[sensorNumber].buffer[i] = bufferCopy[i];
	}
	HAL_TIM_IC_Start_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel, (U32*)(pulseSensor[sensorNumber].buffer), IC_BUF_SIZE);

	// Find the value in question, which is 1 position backwards from the DMA's current position, which is the end of the buffer copy.
	U16 valueInQuestion = bufferCopy[IC_BUF_SIZE - 1];

	if (pulseSensor[sensorNumber].stopped) {	// If we already know we're stopped potentially end early.
		if (valueInQuestion != 0) {	// If we were previously stopped but may be moving again.
			pulseSensor[sensorNumber].stopped = false; // Declare we are no longer stopped and move on.
		} else {
			*(pulseSensor[sensorNumber].resultStoreLocation) = 0; // Otherwise make sure the store location vlaue is 0 and leave.
			return;
		}
	}

	if (pulseSensor[sensorNumber].DMA_lastReadValue == valueInQuestion) {	// Check if the last read value is the same as the current
		if (HAL_GetTick() - pulseSensor[sensorNumber].lastDMAReadValueTimeMs >= DMA_STOPPED_TIMEOUT_MS){	// Check if we haven't changed values in a while which might mean we're stopped
			pulseSensor[sensorNumber].stopped = true;

			// Clear buffer so any non-zero values will be quickly identified that the car is moving again. This will also reset the DMA position.
			HAL_TIM_IC_Stop_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel);
			for (U16 c = 0; c < IC_BUF_SIZE - 1; c++)
			{
				pulseSensor[sensorNumber].buffer[c] = 0;
			}
			HAL_TIM_IC_Start_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel, (U32*)(pulseSensor[sensorNumber].buffer), IC_BUF_SIZE);

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
	U16 value2 = bufferCopy[IC_BUF_SIZE - 2];
	U16 mostRecentDelta = 0;
	// Find the the most rececnt difference between 2 values, with roll-over protection
	if (valueInQuestion < value2) {
		mostRecentDelta = ((1 << 16) | valueInQuestion) - value2;	// TODO: Change to make sure it works with timers of different bit sizes
	} else {
		mostRecentDelta = valueInQuestion - value2;
	}

	U16 amountOfSamples = 10;
//	if(pulseSensor[sensorNumber].useVariableSpeedSampling) {
//		// Equation for getting how many samples we should average, which is linear from low to high samples
//		amountOfSamples = (1 / (float)(pulseSensor[sensorNumber].highTimeDeltaValue)) * mostRecentDelta * IC_BUF_SIZE + pulseSensor[sensorNumber].lowTimeDeltaValue;
//	} else {
//		amountOfSamples = pulseSensor[sensorNumber].highTimeDeltaValue;
//		if (amountOfSamples == 0) {
//			amountOfSamples = IC_BUF_SIZE;
//		}
//	}
//	if (amountOfSamples > IC_BUF_SIZE) {
//		amountOfSamples = IC_BUF_SIZE;
//	}

	// Calculate the deltas between each of the time values and store them in deltaList
	U16 deltaList[IC_BUF_SIZE] = {0};
	for (U16 c = 0; c < amountOfSamples; c++)
	{
		S16 i = (IC_BUF_SIZE - 1 - c);
		U16 value1 = bufferCopy[i];
		U16 value2 = bufferCopy[i - 1];
		if (value1 < value2) {
			deltaList[c] = ((1 << 16) | value1) - value2; // TODO: Change to make sure it works with timers of different bit sizes // Is validated to work
		} else {
			deltaList[c] = value1 - value2;
		}
	}

	// Get average of deltas
	U32 deltaTotal = 0;
	for (U16 c = 0; c < amountOfSamples - 1; c++) {
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

	float result = deltaTotal / (amountOfSamples - 1);

	if(result >= TrackedResult * 1.3 || result <= TrackedResult * 0.7) {
		printf("--- Anomaly Detected! ---\n");
		printf("Current Value: %f\n", result);
		printf("Previous Value: %f\n", TrackedResult);
		printf("DMA Position: %u\n", TrackedDMACurrentPosition);
		printf("Amount of Samples: %u\n", TrackedAmountOfSamples);
		printf("Current Tick: %lu\n", HAL_GetTick());
		printf("Distance from Last Occurence: %ul\n", HAL_GetTick() - lastTick);
		lastTick = HAL_GetTick();
		for (int i = 0; i < IC_BUF_SIZE; i++) {
			printf("Value %i: ", i);
			if (i == DMACurrentPosition) {
				printf("- ");
			}
			printf("%u\n", bufferCopy[i]);
		}
		printf("DELTAS ===\n");
		for (int i = 0; i < IC_BUF_SIZE; i++) {
			printf("Value %i: ", i);
			printf("%u\n", deltaList[(U16)((DMACurrentPosition - i + IC_BUF_SIZE) % IC_BUF_SIZE)]);
		}
	}

	TrackedResult = result;
	pulseSensor[sensorNumber].averageSpeedTimerTicks = result;
	*pulseSensor[sensorNumber].resultStoreLocation = result; //TODO: Check
}
