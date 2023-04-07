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
#include <stdlib.h>
#include <math.h>

#define DEBUG_PS

PulseSensor pulseSensor[TIMER_COUNT] = {0};
U8 numSensors = 0;

static void clear_buffer_and_reset_dma(U8 sensorNumber);
static float convert_delta_time_to_frequency(float deltaTime, float timerPeriodSeconds);

#ifdef DEBUG_PS
// Debug Variables Start
static S16 TrackedDMACurrentPosition = 0;
static U16 TrackedAmountOfSamples = 0;
static U32 TrackedValueInQuestion = 0;
static U32 TrackedDeltaList[IC_BUF_SIZE] = {0};
static U32 TrackedBufferCopy[IC_BUF_SIZE] = {0};
static U32 TrackedDeltaTotal = 0;
static float TrackedResult = 0;
static U32 lastTick = 0;
static bool error = false;
static U16 numLoops = 0;
static float TrackedFrequency = 0;
U32 clockFrequency;
static U32 TrackedValue = 0;
static float TrackedLength = 0;
static U32 lastDeltaTotal = 0;
static U16 lastNumDeltas = 0;
static U16 TrackedNumDuplicateValues = 0;
static U16 TrackedNumDroppedValues = 0;
static U16 numTimesNaNorInf = 0;
static U32 deltaList[MAX_DELTAS] = {0};

// VSS Debug Vars
static U32 TrackedTempDeltaTotal = 0;
static float TrackedMinAverageDelta = 0;
static float TrackedTempFrequencyCalc = 0;

// Debug Variables End
#endif

// Function for setting up timer with all details, use this function directly to include variable speed sampling (vss) data.
void setup_pulse_sensor_vss(
		TIM_HandleTypeDef* htim,
		U32 channel,
		float conversionRatio,
		float* resultStoreLocation,
		U16 dmaStoppedTimeoutMS,
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
	newSensor->dmaStoppedTimeoutMS = dmaStoppedTimeoutMS;
	newSensor->lowPulsesPerSecond = lowPulsesPerSecond;
	newSensor->highPulsesPerSecond = highPulsesPerSecond;
	newSensor->minSamples = minSamples;

	U32 clockFrequency = HAL_RCC_GetSysClockFreq() * 0.5; // Multiply by 0.5 as timers are 1/2 the clock frequency

	// Evaluate the size of the given timer by checking if the Auto reload value is the max size of a 16bit value 0xFFFF
	// TODO: Verify the 32 bit detection works (16 bit validated, so it should work)
	// TODO: Change clock frequency if certain timers
	if (htim->Instance->ARR == 0xFFFF) {
		newSensor->timerSize = 16;
	} else {
		newSensor->timerSize = 32;
	}

	newSensor->timerPeriodSeconds = 1 / ((float)clockFrequency / (htim->Instance->PSC + 1));

	// Clear the sensor's buffer
	memset(newSensor->buffer, 0, sizeof(U32)*IC_BUF_SIZE);

	// Default other values to 0
	newSensor->averageDeltaTimerTicks = 0;
	newSensor->lastDMAReadValueTimeMs = 0;
	newSensor->DMALastReadValue = 0;
	newSensor->stopped = true;
	newSensor->vssSlope = (IC_BUF_SIZE - minSamples) / (float)(highPulsesPerSecond - lowPulsesPerSecond); // Calculate the constant slope value for vss once at the beginning here.

	HAL_TIM_IC_Start_DMA(htim, channel, (U32*)pulseSensor[numSensors].buffer, IC_BUF_SIZE);

	numSensors++;
}

// Function for setting up timer without variable speed sampling (vss)
void setup_pulse_sensor(
		TIM_HandleTypeDef* htim,
		U32 channel,
		float conversionRatio,
		float* resultStoreLocation,
		U16 dmaStoppedTimeoutMS
		)
{
	setup_pulse_sensor_vss(
		htim,
		channel,
		conversionRatio,
		resultStoreLocation,
		dmaStoppedTimeoutMS,
		false,
		0,	// Low RPM Value
		0,	// High RPM Value
		0 	// Min samples
		);
}

// Function for going through all of the currently set up pulse sensors
void check_pulse_sensors() {
	for (int sensorNumber = 0; sensorNumber < numSensors; sensorNumber++) {
		evaluate_pulse_sensor(sensorNumber);
	}
}

// Function that goes through the buffer of the given pulse sensor, handling edge cases, and setting the return value to the found speed value.
void evaluate_pulse_sensor(int sensorNumber) {

	// Get the current tick and use it throughout the whole function so it can't change between operations.
	U32 currentTick = HAL_GetTick();

	// Find the current position of DMA for the current sensor
	S16 DMACurrentPosition = 0;//IC_BUF_SIZE - (U16)((pulseSensor[sensorNumber].htim->hdma[1])->Instance->NDTR);

	// Find the value in question, which is 1 position backwards from the DMA's current position, which is the end of the buffer copy.
	U16 lastBufferPosition = (DMACurrentPosition - 1 + IC_BUF_SIZE) % IC_BUF_SIZE;

	printf("Current Position: %s\n", DMACurrentPosition);
	printf("Last buffer position: %i\n", lastBufferPosition);

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
				return;
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
					return;
				}
			}
		} else {
			*(pulseSensor[sensorNumber].resultStoreLocation) = 0; // Otherwise make sure the store location value is 0 and leave.
			return;
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

			return;
		}
		// Value is the same as the last but it hasn't been enough time to declare stale, hop out of here for now because there's nothing new to process
		return;
	}

	// ===== Passed All Breakpoint Checks =====
	pulseSensor[sensorNumber].DMALastReadValue = valueInQuestion;
	pulseSensor[sensorNumber].lastDMAReadValueTimeMs = currentTick;

	// Declare amount of samples and evaluate initial size based on if we're using variable sampling. If we are we'll determine it again after some deltas are evaluated
	U16 amountOfSamples = pulseSensor[sensorNumber].useVariableSpeedSampling ? pulseSensor[sensorNumber].minSamples + 1 : MAX_DELTAS;

	// Calculate the deltas between each of the time values and store them in deltaTotal
	U32 deltaTotal = 0; // Gets big adding up all U32 deltas but not big enough to need U64 before stopped if like <1 full second
	U32 lastDelta = 0;
	U32 last2ndDelta = 0;
	U16 numDeltas = 0;
	U32 delta = 0;

	for (U16 i = 0; i < amountOfSamples; i++)
	{
		U32 value1 = pulseSensor[sensorNumber].buffer[(DMACurrentPosition - i + IC_BUF_SIZE) % IC_BUF_SIZE];
		U32 value2 = pulseSensor[sensorNumber].buffer[(DMACurrentPosition - i - 1 + IC_BUF_SIZE) % IC_BUF_SIZE];
		if (value1 != 0 && value2 != 0) { // Check if 0s because of previously empty buffer, because otherwise a value would simply be a time amount
			if (abs(value2 - value1) > DUPLICATE_VALUE_TICK_DIFFERENCE) {


				if (value1 < value2) {
					delta = ((1 << pulseSensor[sensorNumber].timerSize) | value1) - value2; // Buffer roll-over (timer resets to 0) protection
				} else {
					delta = value1 - value2; // Buffer roll-over (timer resets to 0) protection
				}
	#ifdef DEBUG_PS
				deltaList[numDeltas] = delta;
	#endif
				// Smoothing for issue where DMA loses values
				if(last2ndDelta == 0) {
					if(lastDelta != 0) {
						if(lastDelta > delta * 1.8) {
							deltaTotal -= lastDelta;
							deltaTotal += delta;
							TrackedNumDroppedValues++;
						}
					}
				} else {
					if ((lastDelta > delta * 1.8) && (lastDelta > last2ndDelta * 1.8)) {
						deltaTotal -= lastDelta;
						lastDelta = (delta + last2ndDelta) * 0.5;
						deltaTotal += lastDelta;
						TrackedNumDroppedValues++;
					}
				}
				deltaTotal += delta;
				last2ndDelta = lastDelta;
				lastDelta = delta;

				numDeltas++;
			}
#ifdef DEBUG_PS
			else {
				TrackedNumDuplicateValues++;
			}
#endif
		}

		// When we're at the minimum samples, take a break to re-evaluate our amount of samples from it's initial value of the minimum + 1.
		if ((i == ((DMACurrentPosition - 1 - pulseSensor[sensorNumber].minSamples + IC_BUF_SIZE) % IC_BUF_SIZE)) && pulseSensor[sensorNumber].useVariableSpeedSampling) {
			// Need a temp total so it doesn't mess up the regular loops bad value rejection
			U32 tempDeltaTotal = deltaTotal;
			// All deltas are protected from dropped values.
			if (delta > last2ndDelta * 1.8)
			{
				tempDeltaTotal -= delta;
				tempDeltaTotal += last2ndDelta;
				TrackedNumDroppedValues++;
			}

			// If we're somehow left with no deltas don't divide by 0
			if (numDeltas != 0) {

				float minSamplesAverageDelta = tempDeltaTotal / (float)numDeltas; // Don't lose accuracy with int devision

				float tempFrequencyCalc = convert_delta_time_to_frequency(minSamplesAverageDelta, pulseSensor[sensorNumber].timerPeriodSeconds);

#ifdef DEBUG_PS
				//Debug
				TrackedTempDeltaTotal = tempDeltaTotal;
				TrackedMinAverageDelta = minSamplesAverageDelta;
				TrackedTempFrequencyCalc = tempFrequencyCalc;
#endif

				if (tempFrequencyCalc < pulseSensor[sensorNumber].lowPulsesPerSecond) {
					amountOfSamples = pulseSensor[sensorNumber].minSamples;
				} else if (tempFrequencyCalc > pulseSensor[sensorNumber].highPulsesPerSecond) {
					amountOfSamples = IC_BUF_SIZE;
				} else {
					// Equation for getting how many samples we should average, which is linear from low to high samples - y = m(x-x0) + y0
					amountOfSamples = pulseSensor[sensorNumber].vssSlope * (tempFrequencyCalc - pulseSensor[sensorNumber].lowPulsesPerSecond) + pulseSensor[sensorNumber].minSamples; // Point slope form lol
				}
			} else {
				amountOfSamples = pulseSensor[sensorNumber].minSamples;
			}
		}
	}

	if (delta > last2ndDelta * 1.8)
	{
		deltaTotal -= delta;
		deltaTotal += last2ndDelta;
		TrackedNumDroppedValues++;
	}

	float result = 0;

	// Calculate average
	float resultingAverageDelta = deltaTotal / (float)numDeltas;

	pulseSensor[sensorNumber].averageDeltaTimerTicks = resultingAverageDelta;

	if(!(isinf(resultingAverageDelta) || isnan(resultingAverageDelta))) {

		// Calculate result from average delta
		result = convert_delta_time_to_frequency(resultingAverageDelta, pulseSensor[sensorNumber].timerPeriodSeconds) * pulseSensor[sensorNumber].conversionRatio;

		// Check if calculation came out inf or nan
		if(isinf(result) || isnan(result)) {
			result = 0;
#ifdef DEBUG_PS
			numTimesNaNorInf++;
#endif
		}
	}
#ifdef DEBUG_PS
	else {
		numTimesNaNorInf++;
	}
#endif

	// Send result to store location
	*pulseSensor[sensorNumber].resultStoreLocation = result;

#ifdef DEBUG_PS
	// Debug Start
	TrackedDMACurrentPosition = DMACurrentPosition;
	TrackedAmountOfSamples = amountOfSamples;
	TrackedValueInQuestion = valueInQuestion;
//	TrackedFirstValue = bufferCopy[IC_BUF_SIZE - 1];
//	TrackedLastValue = bufferCopy[0];
	TrackedDeltaTotal = deltaTotal;
	TrackedFrequency = convert_delta_time_to_frequency(resultingAverageDelta, pulseSensor[sensorNumber].timerPeriodSeconds);
	numLoops++;

	if (error) {
		error = false;
	} else {
		//for(int i = 1; i < numDeltas - 1; i++) {
		//	if(deltaList[i] > deltaList[i-1]*1.45 && deltaList[i] > deltaList[i+1]*1.45){
		if(resultingAverageDelta >= TrackedResult * 1.01 || resultingAverageDelta <= TrackedResult * 0.99) {
				printf("--- Anomaly Detected! ---\n");
				printf("Current Value: %f\n", resultingAverageDelta);
				printf("Previous Value: %f\n", TrackedResult);
				printf("DMA Position: %u\n", TrackedDMACurrentPosition);
				printf("Num Loops: %u\n", numLoops);
				printf("Num Deltas: %u\n", numDeltas);
				printf("Last Num Deltas: %u\n", lastNumDeltas);
				printf("Delta total: %lu\n", deltaTotal);
				printf("Last Delta total: %lu\n", lastDeltaTotal);
				printf("Amount of Samples: %u\n", TrackedAmountOfSamples);
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastTick);
				lastTick = HAL_GetTick();
				for (int i = 0; i < IC_BUF_SIZE; i++) {
					printf("Value %i: ", i);
					if (i == DMACurrentPosition) {
						printf("- ");
					}
					printf("%lu\n", pulseSensor[sensorNumber].buffer[i]);
				}
				printf("DELTAS ===\n");
				for (int i = 0; i < IC_BUF_SIZE; i++) {
					printf("Value %i: ", i);
					printf("%lu\n", deltaList[i]);
					//(U16)((DMACurrentPosition - i + IC_BUF_SIZE) % IC_BUF_SIZE)
				}
				error = true;
				numLoops = 0;
			//}
		}
	}
	lastNumDeltas = numDeltas;
	lastDeltaTotal = deltaTotal;
	TrackedResult = resultingAverageDelta;
	// Debug end
#endif
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
	float frequencyHz = 1 / lengthOfDeltaSeconds;

#ifdef DEBUG_PS
	// Debug
	TrackedValue = deltaTime;
	TrackedLength = lengthOfDeltaSeconds;
#endif

	return frequencyHz;
}
