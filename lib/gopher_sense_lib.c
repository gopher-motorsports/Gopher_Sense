

#include "gopher_sense_lib.h"
#include "gopher_sense.h"
#include "GopherCAN.h"
#include "base_types.h"
#include "main.h"
#include "dam_hw_config.h"
#include "DAM.h"


ADC_HandleTypeDef* adc1 = NULL;
ADC_HandleTypeDef* adc2 = NULL;
ADC_HandleTypeDef* adc3 = NULL;

TIM_HandleTypeDef* adc_timer = NULL;

#define ADC_SAMPLE_SIZE_PER_PARAM 64

#define ADC1_SAMPLE_BUFFER_SIZE NUM_ADC1_PARAMS*ADC_SAMPLE_SIZE_PER_PARAM
#define ADC2_SAMPLE_BUFFER_SIZE NUM_ADC2_PARAMS*ADC_SAMPLE_SIZE_PER_PARAM
#define ADC3_SAMPLE_BUFFER_SIZE NUM_ADC3_PARAMS*ADC_SAMPLE_SIZE_PER_PARAM

volatile U16 adc1_sample_buffer[ADC1_SAMPLE_BUFFER_SIZE];
volatile U16 adc2_sample_buffer[ADC1_SAMPLE_BUFFER_SIZE];
volatile U16 adc3_sample_buffer[ADC1_SAMPLE_BUFFER_SIZE];

#define ADC_VOLTAGE 3.3
#define VOLTAGE_3V3 3.3
#define VOLTAGE_5V  5.0
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
	HAL_TIM_Base_Start_IT(adc_timer);

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
}


// stopDataAq
//  stop running DMA and the timers
// TODO is there any reason to ever stop? Maybe it is ok as long as we are not stopping
// and starting all of the time
void stopDataAq(void)
{
	HAL_TIM_Base_Stop_IT(adc_timer);
	__HAL_TIM_SET_COUNTER(adc_timer, 0);

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
	// config the timer
	if (!tim) return TMR_NOT_CONFIGURED;
    adc_timer = tim;
    configTimer(adc_timer, psc, tim_freq);

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


//******************* CAN Handling *******************

// Redesign option: pull this into a queue and handle not in an ISR
// TODO do the redesign option
void sensor_can_message_handle (CAN_HandleTypeDef* hcan, U32 rx_mailbox)
{
    CAN_RxHeaderTypeDef rx_header;
    CAN_MSG message;
    CAN_SENSOR_PARAM* param = can_sensor_params;
    CAN_SENSOR* sensor;
    SENSOR_CAN_MESSAGE* can_info;

    // Get the message
    if (HAL_CAN_GetRxMessage(hcan, rx_mailbox, &rx_header, message.data) != HAL_OK)
    {
        // Handle errors ?
        return;
    }
    message.rtr_bit = rx_header.RTR;
    message.dlc = rx_header.DLC;
    message.id = (rx_header.IDE ? rx_header.ExtId : rx_header.StdId);

    // Check the CAN params for a match
    while (param - can_sensor_params < NUM_CAN_SENSOR_PARAMS)
    {
        sensor = param->can_sensor;
        can_info = &sensor->messages[param->message_idx];

        // TODO support floating point numbers in CAN messages
        // check for ID match between this param message id and the message id
        if (can_info->message_id == message.id)
        {
            U32 data = 0;
            U8 shift = 0;

            // check that the DLC of this message can support the data we are expecting
            if (message.dlc <= can_info->data_end)
            {
            	handle_DAM_error(SCAN_DLC_ERROR);
            }

            // Get correct data based on byte order
            if (sensor->byte_order == LSB)
            {
                for (U8 b = can_info->data_start; b <= can_info->data_end; b++)
                {
                    data &= message.data[b] << shift;
                    shift += 8;
                }
            }

            else if (sensor->byte_order == MSB)
            {
                for (U8 b = can_info->data_end; b >= can_info->data_start; b--)
                {
                    data &= message.data[b] << shift;
                    shift += 8;
                }
            }
            // Add the data to the buffer
            add_to_buffer(&param->buffer, data);
        }

        param++;
    }
}




//******************* Buffer interaction *******************

// Note: Semaphore probably not needed for buffer interaction because reset is atomic

S8 buffer_full (U32_BUFFER* buffer)
{
    if (buffer == NULL)
    {
        return BUFFER_ERR;
    }
    return buffer->fill_level == buffer->buffer_size;
}

S8 add_to_buffer (U32_BUFFER* buffer, U32 toadd)
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

S8 reset_buffer (U32_BUFFER* buffer)
{
    if (buffer == NULL)
    {
        return BUFFER_ERR;
    }

    buffer->fill_level = 0;
    buffer->buffer_head = 0;
    return BUFFER_SUCCESS;
}

S8 average_buffer (U32_BUFFER* buffer, U32* avg)
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
    	*avg = (U32)(calc_avg / buffer->buffer_size);
    }
    else
    {
    	// the buffer is partially full
    	for (c = 0; c < buffer->fill_level; c++)
    	{
    		calc_avg += buffer->buffer[(buffer->buffer_head + c) % buffer->buffer_size];
    	}
    	*avg = (U32)(calc_avg / buffer->fill_level);
    }

    return BUFFER_SUCCESS;

}


S8 apply_can_sensor_conversion (CAN_SENSOR* sensor, U8 msg_idx, float data_in, float* data_out)
{
	if (!sensor || !data_out) return CONV_ERR;

	DATA_SCALAR scalar = sensor->messages[msg_idx].output.scalar;
	*data_out = (data_in + scalar.offset) * scalar.quantization; // TODO Verify this is the case for all sensors
    return CONV_SUCCESS;
}


S8 apply_analog_sensor_conversion (ANALOG_SENSOR* sensor, float data_in, float* data_out)
{
	if (!sensor || !data_out) return CONV_ERR;

    switch (sensor->model.input_type)
    {
		case VOLTAGE:
			return convert_voltage_load(sensor, data_in, data_out);
		case RESISTIVE:
			return convert_resistive_load(sensor, data_in, data_out);
		case CURRENT:
			return convert_current_load(sensor, data_in, data_out);
		default:
			// apply no conversion
			*data_out = data_in;
    }

    return CONV_ERR;
}


S8 convert_voltage_load (ANALOG_SENSOR* sensor, float data_in, float* data_out)
{
	OUTPUT_MODEL model = sensor->model;
	float v_sensor;

	// Convert the ADC reading to a voltage value
	v_sensor = adc_to_volts(data_in, sensor->output.data_size_bits);

	// Convert the voltage read by the ADC to the voltage read at the pin

	// check to make sure there are no pull-ups. There should not be on a voltage sensor
	if (model.r3v != RES_OPEN || model.r5v != RES_OPEN) return RESISTOR_ERR;

	// check if there is a voltage divider after the amp. If so, modify the voltage
	if (model.rdiv != RES_OPEN && model.rdiv != 0)
	{
		v_sensor = (v_sensor * (model.rfilt + model.rdiv)) / model.rdiv;
	}

	// check if there is voltage divider before the amp. If so, modify the voltage
	if (model.rdown != RES_OPEN && model.rdown != 0)
	{
		v_sensor = (v_sensor * (model.rin + model.rdown)) / model.rdown;
	}

	// find what the sensor read to give that input
	return interpolate_table_linear(sensor->model.table, v_sensor, data_out);
}


S8 convert_resistive_load (ANALOG_SENSOR* sensor, float data_in, float* data_out)
{
	OUTPUT_MODEL model = sensor->model;
	float r_sensor;
	float v_amp;

	// Convert the ADC reading to a voltage value
	v_amp = adc_to_volts(data_in, sensor->output.data_size_bits);

	// Convert the voltage read at the ADC to a resistance read in line with the pin

	// check to make sure there is only one pullup and no rdown
	if (model.rdown != RES_OPEN ||
			(model.r3v != RES_OPEN && model.r5v != RES_OPEN) ||
			(model.r3v == RES_OPEN && model.r5v == RES_OPEN))
	{
		return RESISTOR_ERR;
	}

	// check if there is a voltage divider after the amp. If so, modify the voltage
	if (model.rdiv != RES_OPEN && model.rdiv != 0)
	{
		v_amp = (v_amp * (model.rfilt + model.rdiv)) / model.rdiv;
	}

	// calculate the resistance that creates this voltage at the amp
	if (model.r3v != RES_OPEN)
	{
		r_sensor = ((v_amp * model.r3v) / (VOLTAGE_3V3 - v_amp)) - model.rin;
	}
	else if (model.r5v != RES_OPEN)
	{
		r_sensor = ((v_amp * model.r5v) / (VOLTAGE_5V - v_amp)) - model.rin;
	}
	else
	{
		return RESISTOR_ERR;
	}

	return interpolate_table_linear(sensor->model.table, r_sensor, data_out);
}


S8 convert_current_load (ANALOG_SENSOR* sensor, float data_in, float* data_out)
{
	OUTPUT_MODEL model = sensor->model;
	float ma_sensor;
	float v_amp;

	// Convert the ADC reading to a voltage value
	v_amp = adc_to_volts(data_in, sensor->output.data_size_bits);

	// Convert the voltage read at the ADC to a current sunk into the pin

	// check to make sure there are no pullups and rdown is not open
	if (model.r3v != RES_OPEN || model.r5v != RES_OPEN ||
			model.rdown == RES_OPEN || model.rdown == 0)
	{
		return RESISTOR_ERR;
	}

	// check if there is a voltage divider after the amp. If so, modify the voltage
	if (model.rdiv != RES_OPEN && model.rdiv != 0)
	{
		v_amp = (v_amp * (model.rfilt + model.rdiv)) / model.rdiv;
	}

	// convert the voltage at the amp to a current through rdown (in mA)
	ma_sensor = 1000 * (v_amp / model.rdown);

	return interpolate_table_linear(sensor->model.table, ma_sensor, data_out);
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


// TODO not implemented
S8 apply_special_conversions (ANALOG_SENSOR* sensor, float data_in, float* data_out)
{
	*data_out = data_in;
	return CONV_SUCCESS;
}


// TODO not implemented
S8 apply_filter (U32_BUFFER* buffer, FILTERED_PARAM* filter)
{
    return BUFFER_SUCCESS;
}





