
#include "pulse_sensor.h"
#include "gopher_sense.h"
#include <stdbool.h>

static U16 buffer[IC_BUF_SIZE] = {0};
PulseSensor pulseSensor[TIMER_COUNT] = {0};

static U16 averageSpeedTimerTicks = 0;
static U16 averageSpeedMsPerTick = 0;

// Debug Variables Start
static U32 lastDMACheckMs = 0;
static S16 DMACurrentPosition = 0;
static U16 mostRecentDelta = 0;
static U16 amountOfSamples = 0;
static U16 valueInQuestion = 0;
static U16 firstValue = 0;
static U16 lastValue = 0;
static U16 RPM = 0;
static U16 deltaList[IC_BUF_SIZE] = {0};
static U32 deltaTotal = 0;
static U16 signalRate = 0;
static U16 bufferCopy[IC_BUF_SIZE] = {0};
// Debug Variables End

void setupTimerAndStartDMA(HAL_timer_typdef* htim, TIM_CHANNEL channel, U32 timerPeriodNs, U16 lowSamples, U16 highSamples, U16 ticksPerRev, float conversionRatio, float* resultStoreLocation, bool useVariableSpeedSampling) {
	int timerCount = 0;
	for(Timer t : pulseSensor) {
		if(t.timeR != 0) {
			timerCount++;
		}
	}

	pulseSensor[timerCount].htim = htim;
	pulseSensor[timerCount].channel = channel;
	pulseSensor[timerCount].timerPeriodNs = timerPeriodNs;
	pulseSensor[timerCount].useVariableSpeedSampling = useVariableSpeedSampling;
	pulseSensor[timerCount].lowSamples = lowSamples;
	pulseSensor[timerCount].highSamples = highSamples;
	pulseSensor[timerCount].ticksPerRev = ticksPerRev;
	pulseSensor[timerCount].conversionRatio = conversionRatio;
	pulseSensor[timerCount].resultStoreLocation = resultStoreLocation;
	pulseSensor[timerCount].buffer = {0};

	HAL_TIM_IC_Start_DMA(htim, channel, (U32*)pulseSensor[timerCount].buffer, IC_BUF_SIZE);
}

void checkTransSpeedDMAs() {
	int activeTimerCount = 0;
	for(Timer t : pulseSensor) {
		if(t.timeR != 0) {
			activeTimerCount++;
		}
	}

	for (int sensorNumber = 0; sensorNumber < activeTimerCount; sensorNumber++) {

		//if (HAL_GetTick() - pulseSensor[sensorNumber].lastDMACheckMs >= timerPeriodNs) {

			// TODO: Figure out how to set DMA postiion so dma can be stoppedwithout losing position
			//HAL_TIM_IC_Stop_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel);

			// Copy all the values so they can't change while doing calculations
			//U16 bufferCopy[IC_BUF_SIZE] = {0};
			//U16 DMACurrentPosition = IC_BUF_SIZE - (U16)((pulseSensor[sensorNumber].htim->hdma[1])->Instance->NDTR);
			DMACurrentPosition = IC_BUF_SIZE - (U16)((pulseSensor[sensorNumber].htim->hdma[1])->Instance->NDTR);
			for (U16 c = 0; c < IC_BUF_SIZE; c++)
			{
				bufferCopy[c] = pulseSensor[sensorNumber].buffer[c];
			}
			//HAL_TIM_IC_Start_DMA(pulseSensor[sensorNumber].htim, pulseSensor[sensorNumber].channel, (U32*)(pulseSensor[sensorNumber].buffer), IC_BUF_SIZE);

			//U16 valueInQuestion = bufferCopy[(U16)((DMACurrentPosition - 1 + IC_BUF_SIZE) % IC_BUF_SIZE)];
			valueInQuestion = bufferCopy[(U16)((DMACurrentPosition - 1 + IC_BUF_SIZE) % IC_BUF_SIZE)];

			if (pulseSensor[sensorNumber].stopped) {	// If we know we're stopped potentially end early
				if (valueInQuestion != 0) {	// If we were previously stopped but may be moving again
					stopped = false;
				} else {
					return 0;
				}
			}

			if (pulseSensor[sensorNumber].DMA_lastReadValue == valueInQuestion) {	// Check if the last read value is the same as the current
				if (HAL_GetTick() - pulseSensor[sensorNumber].lastDMAReadValueTimeMs >= DMA_STOPPED_TIMEOUT){	// Check if we haven't changed values in a while which might mean we stopped
					pulseSensor[sensorNumber].stopped = true;

					// Clear buffer so any non-zero values will be quickly identified that the car is moving again
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
			// TODO: Protect this from rollover, and make sure there aren't other issues
			U16 value2 = bufferCopy[(U16)((DMACurrentPosition - 2 + IC_BUF_SIZE) % IC_BUF_SIZE)];
			if (valueInQuestion < value2) {
				mostRecentDelta = ((1 << 16) | valueInQuestion) - value2;
			} else {
				mostRecentDelta = valueInQuestion - value2;
			}
			//U16 amountOfSamples = 40;// (DMA_HIGH_SAMPLES / HIGH_RPM) * mostRecentDelta + DMA_LOW_SAMPLES; // Equation for getting how many samples we should average
			amountOfSamples = 40;

			// Calculate the deltas between each of the time values and store them in deltaList
			//U16 deltaList[IC_BUF_SIZE] = {0};
			for (U16 c = 0; c < amountOfSamples - 1; c++)
			{
				S16 i = (DMACurrentPosition - 1 - c + IC_BUF_SIZE) % IC_BUF_SIZE;
				U16 value1 = bufferCopy[i];
				U16 value2 = bufferCopy[(i - 1 + IC_BUF_SIZE) % IC_BUF_SIZE];
				if (value1 < value2) {
					deltaList[c] = ((1 << 16) | value1) - value2;
				} else {
					deltaList[c] = value1 - value2;
				}
			}

			// Get average of deltas
			//U32 deltaTotal = 0;
			deltaTotal = 0;
			for (U16 c = 0; c < amountOfSamples - 1; c++) {
				deltaTotal += deltaList[c];
			}

			// Debug Start
			firstValue = bufferCopy[0];
			lastValue = bufferCopy[IC_BUF_SIZE - 1];
			// Debug end

			pulseSensor[sensorNumber].lastDMAReadValueTimeMs = HAL_GetTick();
		}
	}

//	U16 result = deltaTotal / (amountOfSamples - 1);
	U16 result = deltaTotal / (amountOfSamples - 1);
	pulseSensor[sensorNumber].averageSpeedTimerTicks = result;
	pulseSensor[sensorNumber].resultStoreLocation = result;
	return;
}