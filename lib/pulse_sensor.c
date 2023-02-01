
#include <stdbool.h>

static U16 ic1buf[IC_BUF_SIZE] = {0};
Timer timer[TIMER_COUNT] = {0};

static U16 averageSpeedTimerTicks = 0;

static U16 lowSamples;
static U16 highSamples;
static U16 ticksPerRevSamples;
#define MS_IN_A_MINUTE 60000

timer_struct_t timer_struct;


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
static U16 ic1bufCopy[IC_BUF_SIZE] = {0};
// Debug Variables End

static void setupTimerAndStartDMA(HAL_timer_typdef* htim, TIM_CHANNEL channel, U16 lowSamples, U16 highSamples, U16 ticksPerRev) {
	HAL_timer_typdef* timer_struct;

	HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1, (U32*)ic1buf, IC_BUF_SIZE);
}

static U16 checkTransSpeedDMA(timer_struct timStruct, U32 timer_period_ns, float conversion_ratio, float* result_store_location) {

	static U32 lastDMAReadValueTimeMs = 0;
	static U16 DMA_lastReadValue = 0;

	//U16 DMACurrentPosition = 0;

	// Copy all the values so they can't change while doing calculations
	//HAL_TIM_IC_Stop_DMA(&htim3, TIM_CHANNEL_1);
	DMACurrentPosition = IC_BUF_SIZE - (U16)((htim->hdma[1])->Instance->NDTR);
	for (U16 c = 0; c < IC_BUF_SIZE; c++)
	{
		ic1bufCopy[c] = ic1buf[c];
	}
	//HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1, (U32*)ic1buf, IC_BUF_SIZE);

	valueInQuestion = ic1bufCopy[(U16)((DMACurrentPosition - 1 + IC_BUF_SIZE) % IC_BUF_SIZE)];

	if (stopped) {
		if (valueInQuestion != 0) {
			stopped = false;
		} else {
			return 0;
		}
	}
//
//	// Check if the last read value is the same as the current
	if (DMA_lastReadValue == valueInQuestion) {
		// Check if we haven't changed values in a while which might mean we stopped
		if (HAL_GetTick() - *lastDMAReadValueTimeMs >= DMA_STOPPED_TIMEOUT){
			stopped = true;

			// Clear buffer so any non-zero values will be quickly identified that the car is moving again
			HAL_TIM_IC_Stop_DMA(htim, channel);
			for (U16 c = 0; c < IC_BUF_SIZE - 1; c++)
			{
				ic1buf[c] = 0;

			}
			HAL_TIM_IC_Start_DMA(htim, channel, (U32*)ic1buf, IC_BUF_SIZE);

			return 0;
		}
		return averageSpeedMsPerTick;
	}
	DMA_lastReadValue = valueInQuestion;

	// Calculate the amount of samples to take to get the speed based on the delta between the last 2 values
	// TODO: Protect this from rollover, and make sure there aren't other issues
	U16 value2 = ic1bufCopy[(U16)((DMACurrentPosition - 2 + IC_BUF_SIZE) % IC_BUF_SIZE)];
	if (valueInQuestion < value2) {
		mostRecentDelta = ((1 << 16) | valueInQuestion) - value2;
	} else {
		mostRecentDelta = valueInQuestion - value2;
	}
	/*U16*/ //mostRecentDelta = ic1bufCopy[(DMACurrentPosition - 2 + IC_BUF_SIZE) % IC_BUF_SIZE] - valueInQuestion;
	/*U16*/ amountOfSamples = 40;// (DMA_HIGH_SAMPLES / HIGH_RPM) * mostRecentDelta + DMA_LOW_SAMPLES; // Equation for getting how many samples we should average

	// Calculate the deltas between each of the time values and store them in deltaList
	//deltaList[IC_BUF_SIZE] = {0};
	for (U16 c = 0; c < amountOfSamples - 1; c++)
	{
		S16 i = (DMACurrentPosition - 1 - c + IC_BUF_SIZE) % IC_BUF_SIZE;
		U16 value1 = ic1bufCopy[i];
		U16 value2 = ic1bufCopy[(i - 1 + IC_BUF_SIZE) % IC_BUF_SIZE];
		if (value1 < value2) {
			deltaList[c] = ((1 << 16) | value1) - value2;
		} else {
			deltaList[c] = value1 - value2;
		}
	}

	// Get average of deltas
	deltaTotal = 0;
	for (U16 c = 0; c < amountOfSamples - 1; c++) {
		deltaTotal += deltaList[c];
	}

	// Debug Start
	firstValue = ic1bufCopy[0];
	lastValue = ic1bufCopy[IC_BUF_SIZE - 1];

	//	printf("Last DMA Read Time: %lu", *last_DMA_read);
	//	printf("DMA Current Position: %u", DMACurrentPosition);
	//	printf("First array value: %u" + ic1bufCopy[0]);
	//	printf("Last array value: %u" + ic1bufCopy[IC_BUF_SIZE - 1]);
	//	printf("Value in question: %u" + valueInQuestion);
	//	printf("Most recent delta: %u" + mostRecentDelta);
	//	printf("Amount of samples: " + amountOfSamples);

	// Debug End



	*lastDMAReadValueTimeMs = HAL_GetTick();

	// TODO: Translate value to desired units

	return deltaTotal / (amountOfSamples - 1);
}
