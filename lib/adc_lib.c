// adc_lib.c
//  Functions for handling the ADC buffers, DMA, and hardware timers

#include "adc_lib.h"
#include "gopher_sense.h"
#include "GopherCAN.h"
#include "module_hw_config.h"

#if NEED_ADC
ADC_HandleTypeDef* adc1 = NULL;
ADC_HandleTypeDef* adc2 = NULL;
ADC_HandleTypeDef* adc3 = NULL;
#endif

#if NEED_HW_TIMER
TIM_HandleTypeDef* adc_timer = NULL;
#endif

// The number of samples from the DMA buffer averaged into the parameter
// buffer
#define ADC_SAMPLE_SIZE_PER_PARAM 64

#define ADC1_SAMPLE_BUFFER_SIZE NUM_ADC1_PARAMS*ADC_SAMPLE_SIZE_PER_PARAM
#define ADC2_SAMPLE_BUFFER_SIZE NUM_ADC2_PARAMS*ADC_SAMPLE_SIZE_PER_PARAM
#define ADC3_SAMPLE_BUFFER_SIZE NUM_ADC3_PARAMS*ADC_SAMPLE_SIZE_PER_PARAM

#if NUM_ADC1_PARAMS > 0
volatile U16 adc1_sample_buffer[ADC1_SAMPLE_BUFFER_SIZE] = {0};
#endif
#if NUM_ADC2_PARAMS > 0
volatile U16 adc2_sample_buffer[ADC2_SAMPLE_BUFFER_SIZE] = {0};
#endif
#if NUM_ADC3_PARAMS > 0
volatile U16 adc3_sample_buffer[ADC3_SAMPLE_BUFFER_SIZE] = {0};
#endif

#define ADC_BITS    12
#define ADC_VOLTAGE 3.3
#define TIM_CLOCK_BASE_FREQ (HAL_RCC_GetPCLK1Freq() << 1) // APB1 Timer clock = PCLK1 * 2
#define TIM_MAX_VAL (1<<16)
#define RES_SENSOR_PULL_UP_RESISTANCE_R 1000.0
#define RES_SENSOR_PULL_UP_VOLTAGE_V 3.3

// static function declarations
#if NEED_HW_TIMER
static void configTimer(TIM_HandleTypeDef* timer, U16 timer_int_freq_hz, U16 psc);
#endif // NEED_HW_TIMER
static S8 convert_voltage_load(ANALOG_SENSOR* sensor, float voltage, float* data_out);
static S8 convert_resistive_load(ANALOG_SENSOR* sensor, float voltage, float* data_out);
static inline float adc_to_volts(U16 adc_reading, U8 resolution_bits);
static S8 interpolate_table_linear(TABLE* table, float data_in, float* data_out);
static inline float interpolate(float x0, float y0, float x1, float y1, float x);


//******************* ADC Config *******************
#if NEED_ADC
S8 configLibADC(ADC_HandleTypeDef* ad1, ADC_HandleTypeDef* ad2,
		        ADC_HandleTypeDef* ad3)
{
#if NUM_ADC1_PARAMS > 0
    if (!ad1) return ADC_NOT_CONFIGURED;
    adc1 = ad1;
#endif // NUM_ADC1_PARAMS > 0
#if NUM_ADC2_PARAMS > 0
    if (!ad2) return ADC_NOT_CONFIGURED;
    adc2 = ad2;
#endif // NUM_ADC2_PARAMS > 0
#if NUM_ADC3_PARAMS > 0
    if (!ad3) return ADC_NOT_CONFIGURED;
    adc3 = ad3;
#endif // NUM_ADC3_PARAMS > 0

    return ADC_LIB_CONFIG_SUCCESS;
}
#endif


// startDataAq
//  Start the timer and the ADC DMA. Data is constantly filled in the buffer with the DMA
//  Then the most recent sample will be added to the buffer each time the timer is fired
void startDataAq(void)
{
    // start the DMA
#if NUM_ADC1_PARAMS > 0
    HAL_ADC_Start_DMA(adc1, (uint32_t*)adc1_sample_buffer, ADC1_SAMPLE_BUFFER_SIZE);
#endif // NUM_ADC1_PARAMS > 0
#if NUM_ADC2_PARAMS > 0
    HAL_ADC_Start_DMA(adc2, (uint32_t*)adc2_sample_buffer, ADC2_SAMPLE_BUFFER_SIZE);
#endif // NUM_ADC2_PARAMS > 0
#if NUM_ADC3_PARAMS > 0
    HAL_ADC_Start_DMA(adc3, (uint32_t*)adc3_sample_buffer, ADC3_SAMPLE_BUFFER_SIZE);
#endif // NUM_ADC3_PARAMS > 0

    // start the timers to begin moving data to the param buffers
#if NEED_HW_TIMER
    __HAL_TIM_SET_COUNTER(adc_timer, 0);
    HAL_TIM_Base_Start_IT(adc_timer);
#endif
}


// stopDataAq
//  stop running DMA and the timers
void stopDataAq(void)
{
#if NEED_HW_TIMER
	HAL_TIM_Base_Stop_IT(adc_timer);
	__HAL_TIM_SET_COUNTER(adc_timer, 0);
#endif

#if NUM_ADC1_PARAMS > 0
    HAL_ADC_Stop_DMA(adc1);
#endif // NUM_ADC1_PARAMS > 0
#if NUM_ADC2_PARAMS > 0
    HAL_ADC_Stop_DMA(adc2);
#endif // NUM_ADC2_PARAMS > 0
#if NUM_ADC3_PARAMS > 0
    HAL_ADC_Stop_DMA(adc3);
#endif // NUM_ADC3_PARAMS > 0
}


// DAQ_TimerCallback
//  This is called by each timer when it overflows. This will take the data from the DMA
//  buffer and put it in the parameter buffer
#if NEED_HW_TIMER
void DAQ_TimerCallback(TIM_HandleTypeDef* timer)
{
	// make sure the correct timer is being used
	if (timer != adc_timer) return;

	// put the data into the parameter buffer
#if NUM_ADC1_PARAMS > 0
    add_data_to_buffer(adc1_sensor_params, adc1_sample_buffer, NUM_ADC1_PARAMS);
#endif // NUM_ADC1_PARAMS > 0
#if NUM_ADC2_PARAMS > 0
    add_data_to_buffer(adc2_sensor_params, adc2_sample_buffer, NUM_ADC2_PARAMS);
#endif // NUM_ADC2_PARAMS > 0
#if NUM_ADC3_PARAMS > 0
    add_data_to_buffer(adc3_sensor_params, adc3_sample_buffer, NUM_ADC3_PARAMS);
#endif // NUM_ADC3_PARAMS > 0
}
#endif


// add_data_to_buffer
//  for each parameter in this array, transfer the data from the sample buffer
//  to the param buffer
void add_data_to_buffer(ANALOG_SENSOR_PARAM* param_array,
		                volatile U16* sample_buffer, U32 num_params)
{
	ANALOG_SENSOR_PARAM* param;
	volatile U16* buffer = sample_buffer;
	U32 total = 0;

	// run through the DMA buffer and add up all of the samples to be averaged. The prameter
	// samples are offset by the number of parameters in that ADC as the ADC goes through each
	// channel one at a time
	for (param = param_array; param - param_array < num_params; param++)
	{
		// get all the samples for this parameter
		total = 0;
		for (buffer = sample_buffer + (param - param_array);
			 buffer - sample_buffer < (ADC_SAMPLE_SIZE_PER_PARAM*num_params);
			 buffer += num_params)
		{
			total += *buffer;
		}

		// calculate the average and add it to the buffer
		add_to_buffer(&param->buffer, (U16)(total / ADC_SAMPLE_SIZE_PER_PARAM));
	}
}



//******************* Timer interaction *******************

// configLibTimer
//  to be called externally, this function will correctly set up the timer to run
//  at the inputed frequency
#if NEED_HW_TIMER
S8 configLibTIM(TIM_HandleTypeDef* tim, U16 tim_freq, U16 psc)
{
	// config the timer
	if (!tim) return TMR_NOT_CONFIGURED;
    adc_timer = tim;
    configTimer(adc_timer, tim_freq, psc);

    return ADC_LIB_CONFIG_SUCCESS;
}
#endif


// configTimer
//  static function, this will set up the timer with no error checking
#if NEED_HW_TIMER
static void configTimer(TIM_HandleTypeDef* timer, U16 timer_int_freq_hz, U16 psc)
{
    __HAL_TIM_DISABLE(timer);
    __HAL_TIM_SET_COUNTER(timer, 0);
    U32 reload;

    do {
        reload = (U32)((TIM_CLOCK_BASE_FREQ/psc) / timer_int_freq_hz);
        psc <<= 1;
    } while (reload > TIM_MAX_VAL);

    __HAL_TIM_SET_PRESCALER(timer, psc);
    __HAL_TIM_SET_AUTORELOAD(timer, reload);
    __HAL_TIM_ENABLE(timer);
}
#endif // NEED_HW_TIMER


//******************* Buffer interaction *******************

// Note: Semaphore probably not needed for buffer interaction because reset is atomic

// buffer_full
//  pass in a buffer, returns 1 if it is full, 0 if it is not full
S8 buffer_full(U16_BUFFER* buffer)
{
    if (buffer == NULL)
    {
        return BUFFER_ERR;
    }
    return buffer->fill_level == buffer->buffer_size;
}

// add_to_buffer
//  adds a value to the buffer at the next available spot. This is a ring
//  buffer, so if no spots are available this will delete the oldest data
//  in the buffer
S8 add_to_buffer(U16_BUFFER* buffer, U16 toadd)
{
    if (buffer == NULL)
    {
        return BUFFER_ERR;
    }

    if (buffer->fill_level == buffer->buffer_size)
    {
    	// if the buffer is full, replace the head and move it
    	buffer->buffer[buffer->buffer_head] = toadd;
    	buffer->buffer_head = (buffer->buffer_head + 1) % buffer->buffer_size;
    }
    else
    {
    	// buffer is not full, add to the end and increase the fill level
    	buffer->buffer[(buffer->buffer_head + buffer->fill_level)
					   % buffer->buffer_size] = toadd;
    	buffer->fill_level++;
    }

    return BUFFER_SUCCESS;
}

// reset_buffer
//  Set the head and fill level of the buffer to zero in order to consider the
//  buffer empty
S8 reset_buffer(U16_BUFFER* buffer)
{
    if (buffer == NULL)
    {
        return BUFFER_ERR;
    }

    buffer->fill_level = 0;
    buffer->buffer_head = 0;
    return BUFFER_SUCCESS;
}

// average_buffer
//  average the buffer and put the number into avg. Will truncate to the nearest
//  integer
S8 average_buffer(U16_BUFFER* buffer, U16* avg)
{
    if (buffer == NULL)
    {
        return BUFFER_ERR;
    }

    U32 calc_avg = 0;
    U16 c;

    if (buffer->fill_level == buffer->buffer_size)
	{
		// if the buffer is full, average up the entire buffer
		for (c = 0; c < buffer->buffer_size; c++)
		{
			calc_avg += buffer->buffer[c];
		}
		*avg = (U16)(calc_avg / buffer->buffer_size);
	}
    else if (buffer->fill_level == 0)
    {
    	// just set the average to 0 if the buffer is empty
    	*avg = 0;
    	return BUFFER_EMPTY;
    }
    else
    {
    	// the buffer is partially full
    	for (c = 0; c < buffer->fill_level; c++)
    	{
    		calc_avg += buffer->buffer[(buffer->buffer_head + c) % buffer->buffer_size];
    	}
    	*avg = (U16)(calc_avg / buffer->fill_level);
    }

    return BUFFER_SUCCESS;
}


// apply_analog_sensor_conversion
//  this will take a ADC integer value from a sensor and convert it to a
//  real-world value
S8 apply_analog_sensor_conversion(ANALOG_SENSOR* sensor,
		                          U16 data_in, float* data_out)
{
	float voltage = adc_to_volts(data_in, ADC_BITS);

	if (!sensor || !data_out) return CONV_ERR;

    switch (sensor->type)
    {
		case VOLTAGE:
			return convert_voltage_load(sensor, voltage, data_out);
		case RESISTIVE:
			return convert_resistive_load(sensor, voltage, data_out);
		default:
			// apply no conversion, just give out the voltage
			*data_out = voltage;
    }

    return CONV_ERR;
}


// convert_voltage_load
//  take in a raw voltage value and return the real-world value based on
//  the sensor and the data in the voltage conversion table
static S8 convert_voltage_load(ANALOG_SENSOR* sensor, float voltage, float* data_out)
{
	// we just need to convert directly from the voltage to the output value
	return interpolate_table_linear(sensor->conversion_table, voltage, data_out);
}


// convert_resistive_load
//  take in a raw voltage value and return the real-world value based on
//  the sensor and the data in the resistance conversion table
static S8 convert_resistive_load(ANALOG_SENSOR* sensor, float voltage, float* data_out)
{
	float res;
	// convert the voltage to a resistance with some math
	res = (voltage * RES_SENSOR_PULL_UP_RESISTANCE_R) /
			(RES_SENSOR_PULL_UP_VOLTAGE_V - voltage);

	return interpolate_table_linear(sensor->conversion_table, res, data_out);
}


// adc_to_volts
//  do some math to transform the ADC reading into a voltage
static inline float adc_to_volts(U16 adc_reading, U8 resolution_bits)
{
	return ((float)adc_reading / (1 << resolution_bits)) * ADC_VOLTAGE;
}


// interpolate_table_linear
//  take a table, and convert the input value to an output value based on the
//  information and points stored in the table
static S8 interpolate_table_linear(TABLE* table, float data_in, float* data_out)
{
	U16 entries = table->num_entries;
	float x0, x1, y0, y1;

	if (!table || !entries)
	{
		*data_out = data_in;
		return CONV_ERR;
	}

	if (data_in < table->independent_vars[0])
	{
		// linearly interpolate based on the bottom two points
		x0 = table->independent_vars[0];
		y0 = table->dependent_vars[0];
		x1 = table->independent_vars[1];
		y1 = table->dependent_vars[1];
	}
	else if (data_in > table->independent_vars[entries-1])
	{
		// linearly interpolate based on the top two points
		x0 = table->independent_vars[entries - 2];
		y0 = table->dependent_vars[entries - 2];
		x1 = table->independent_vars[entries - 1];
		y1 = table->dependent_vars[entries - 1];
	}
	else
	{
		// scan through all of the points in the table and find which two points
		// this input is between
		for (U16 i = 0; i < entries-1; i++)
		{
			x0 = table->independent_vars[i];
			y0 = table->dependent_vars[i];
			x1 = table->independent_vars[i+1];
			y1 = table->dependent_vars[i+1];

			if (data_in >= x0 && data_in <= x1)
			{
				break;
			}

		}
	}

	// defaults are 0, 0, 3.3, 3.3 so this will just give a voltage if there
	// is some issue getting the conversion
	*data_out = interpolate(x0, y0, x1, y1, data_in);
	return CONV_SUCCESS;
}

// interpolate
//  linearly interpolate between two points
static inline float interpolate(float x0, float y0, float x1, float y1, float x)
{
	return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0);
}


// End of adc_lib.c
