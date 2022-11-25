// adc_lib.c
// TODO DOCS

#include <adc_lib.h>
#include <gopher_sense.h>
#include "gopher_sense.h"
#include "GopherCAN.h"
#include "base_types.h"
#include "main.h"
#include "dam_hw_config.h"


ADC_HandleTypeDef* adc1 = NULL;
ADC_HandleTypeDef* adc2 = NULL;
ADC_HandleTypeDef* adc3 = NULL;

#if NEED_HW_TIMER
TIM_HandleTypeDef* adc_timer = NULL;
#endif

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
#define TIM_MAX_VAL 65536


//******************* ADC Config *******************
S8 configLibADC(ADC_HandleTypeDef* ad1, ADC_HandleTypeDef* ad2, ADC_HandleTypeDef* ad3)
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

    return 0;
}


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
void DAQ_TimerCallback (TIM_HandleTypeDef* timer)
{
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


// for each parameter in this array, transfer the data from the sample buffer to the param buffer
void add_data_to_buffer(ANALOG_SENSOR_PARAM* param_array, volatile U16* sample_buffer, U32 num_params)
{
	ANALOG_SENSOR_PARAM* param = param_array;
	volatile U16* buffer = sample_buffer;
	U32 total = 0;

	// run through the DMA buffer and add up all of the samples to be averaged. The prameter
	// samples are offset by the number of parameters in that ADC as the ADC goes through each
	// channel one at a time
	while (param - param_array < num_params)
	{
		// get all the samples for this parameter
		total = 0;
		buffer = sample_buffer + (param - param_array);
		while (buffer - sample_buffer < (ADC_SAMPLE_SIZE_PER_PARAM*num_params))
		{
			total += *buffer;
			buffer += num_params;
		}

		// calculate the average and add it to the buffer
		add_to_buffer(&param->buffer, (U32)(total / ADC_SAMPLE_SIZE_PER_PARAM));

		// move on to the next param
		param++;
	}
}



//******************* Timer interaction *******************
S8 configLibTIM(TIM_HandleTypeDef* tim, U16 tim_freq, U16 psc)
{
#if NEED_HW_TIMER
	// config the timer
	if (!tim) return TMR_NOT_CONFIGURED;
    adc_timer = tim;
    configTimer(adc_timer, psc, tim_freq);
#endif

    return 0;
}


void configTimer(TIM_HandleTypeDef* timer, U16 psc,  U16 timer_int_freq_hz)
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


//******************* Buffer interaction *******************

// Note: Semaphore probably not needed for buffer interaction because reset is atomic

S8 buffer_full (U16_BUFFER* buffer)
{
    if (buffer == NULL)
    {
        return BUFFER_ERR;
    }
    return buffer->fill_level == buffer->buffer_size;
}

S8 add_to_buffer (U16_BUFFER* buffer, U32 toadd)
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

S8 reset_buffer (U16_BUFFER* buffer)
{
    if (buffer == NULL)
    {
        return BUFFER_ERR;
    }

    buffer->fill_level = 0;
    buffer->buffer_head = 0;
    return BUFFER_SUCCESS;
}

S8 average_buffer (U16_BUFFER* buffer, U16* avg)
{
    if (buffer == NULL)
    {
        return BUFFER_ERR;
    }

    U64 calc_avg = 0;
    U16 c;

    if (buffer->fill_level == 0)
    {
    	// just set the average to 0 if the buffer is empty
    	*avg = 0;
    	return BUFFER_EMPTY;
    }
    else if (buffer->fill_level == buffer->buffer_size)
    {
    	// if the buffer is full, average up the entire buffer
    	for (c = 0; c < buffer->buffer_size; c++)
    	{
    		calc_avg += buffer->buffer[c];
    	}
    	*avg = (U16)(calc_avg / buffer->buffer_size);
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


S8 average_buffer_as_float (U16_BUFFER* buffer, float* avg)
{
	if (buffer == NULL)
	{
		return BUFFER_ERR;
	}

	double calc_avg = 0;
	FLOAT_CONVERTER flt_con;
	U16 c;

	if (buffer->fill_level == 0)
	{
		// just set the average to 0 if the buffer is empty
		*avg = 0;
		return BUFFER_EMPTY;
	}
	else if (buffer->fill_level == buffer->buffer_size)
	{
		// if the buffer is full, average up the entire buffer
		for (c = 0; c < buffer->buffer_size; c++)
		{
			flt_con.u32 = buffer->buffer[c];
			calc_avg += flt_con.f;
		}
		*avg = (float)(calc_avg / buffer->buffer_size);
	}
	else
	{
		// the buffer is partially full
		for (c = 0; c < buffer->fill_level; c++)
		{
			flt_con.u32 = buffer->buffer[(buffer->buffer_head + c) % buffer->buffer_size];
			calc_avg += flt_con.f;
		}
		*avg = (float)(calc_avg / buffer->fill_level);
	}

	return BUFFER_SUCCESS;
}


S8 apply_analog_sensor_conversion (ANALOG_SENSOR* sensor, float data_in, float* data_out)
{
	// TODO this function needs some help
	if (!sensor || !data_out) return CONV_ERR;

    switch (sensor->model.input_type)
    {
		case VOLTAGE:
			return convert_voltage_load(sensor, data_in, data_out);
		case RESISTIVE:
			return convert_resistive_load(sensor, data_in, data_out);
		default:
			// apply no conversion
			*data_out = data_in;
    }

    return CONV_ERR;
}


S8 convert_voltage_load (ANALOG_SENSOR* sensor, float data_in, float* data_out)
{
	// TODO math based on no voltage divider
}


S8 convert_resistive_load (ANALOG_SENSOR* sensor, float data_in, float* data_out)
{
	// TODO math based on a 1k resistor
}


inline float adc_to_volts(U16 adc_reading, U8 resolution_bits)
{
	return ((float)adc_reading / (1 << resolution_bits)) * ADC_VOLTAGE;
}


S8 interpolate_table_linear (TABLE* table, float data_in, float* data_out)
{
	U16 entries = table->num_entries;
	if (!table || !entries)
	{
		*data_out = data_in;
		return CONV_ERR;
	}

	if (data_in < table->independent_vars[0])
	{
		// if off bottom edge return bottom val
		*data_out = table->dependent_vars[0];
		return CONV_SUCCESS;
	}

	if (data_in > table->independent_vars[entries-1])
	{
		// if off top edge return top val
		*data_out = table->dependent_vars[entries-1];
		return CONV_SUCCESS;
	}

	for (U16 i = 0; i < entries-1; i++)
	{
		float x0 = table->independent_vars[i];
		float y0 = table->dependent_vars[i];
		float x1 = table->independent_vars[i+1];
		float y1 = table->dependent_vars[i+1];

		if (data_in >= x0 && data_in <= x1) {
			*data_out = interpolate(x0, y0, x1, y1, data_in);
			return CONV_SUCCESS;
		}

	}

	*data_out = data_in;
	return CONV_ERR;
}


inline float interpolate(float x0, float y0, float x1, float y1, float x)
{
	return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0);
}

