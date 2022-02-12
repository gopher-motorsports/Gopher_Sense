#include "gopher_sense_lib.h"
#include "gopher_sense.h"
#include "GopherCAN.h"
#include "base_types.h"
#include "main.h"

// INCLUDE YOUR HW CONFIG FILE HERE
#include "dam_hw_config.h"


ADC_HandleTypeDef* adc1;
ADC_HandleTypeDef* adc2;
ADC_HandleTypeDef* adc3;

TIM_HandleTypeDef* adc1_timer;
TIM_HandleTypeDef* adc2_timer;
TIM_HandleTypeDef* adc3_timer;

//DMA_HandleTypeDef hdma_adc1;
//DMA_HandleTypeDef hdma_adc2;
//DMA_HandleTypeDef hdma_adc3;

static volatile U16 adc1_sample_buffer[NUM_ADC1_PARAMS];
static volatile U16 adc2_sample_buffer[NUM_ADC2_PARAMS];
static volatile U16 adc3_sample_buffer[NUM_ADC3_PARAMS];

#define ADC_VOLTAGE 3.3
#define TIM_CLOCK_BASE_FREQ 16000000
#define TIM_MAX_VAL 65536


//************ Initialize the library ****************
void init_sensor_hal (void) {
    init_analog_sensors();
    init_can_sensors();
    init_adc1_params();
    init_adc2_params();
    init_adc3_params();
    init_can_params();
    init_buckets();
}

//******************* ADC Config *******************
void configLibADC(ADC_HandleTypeDef* ad1, ADC_HandleTypeDef* ad2, ADC_HandleTypeDef* ad3)
{
    adc1 = ad1;
    adc2 = ad2;
    adc3 = ad3;
}

void  HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc_handle){
    // stop the DMA and start the timer
    HAL_ADC_Stop_DMA(adc_handle);
    U16_BUFFER* buf;

    if (adc_handle == adc1) {
        __HAL_TIM_SET_COUNTER(adc1_timer, 0);
        HAL_TIM_Base_Start_IT(adc1_timer);
        for (U8 i = 0; i < NUM_ADC1_PARAMS; i++) {
            buf = &adc1_sensor_params[i].buffer;
            add_to_buffer(buf, adc1_sample_buffer[i]);
        }
    }
    else if (adc_handle == adc2) {
        __HAL_TIM_SET_COUNTER(adc2_timer, 0);
        HAL_TIM_Base_Start_IT(adc2_timer);
        for (U8 i = 0; i < NUM_ADC2_PARAMS; i++) {
            buf = &adc2_sensor_params[i].buffer;
            add_to_buffer(buf, adc2_sample_buffer[i]);
        }
    }
    else if (adc_handle == adc3) {
        __HAL_TIM_SET_COUNTER(adc3_timer, 0);
        HAL_TIM_Base_Start_IT(adc3_timer);
        for (U8 i = 0; i < NUM_ADC3_PARAMS; i++) {
            buf = &adc3_sensor_params[i].buffer;
            add_to_buffer(buf, adc3_sample_buffer[i]);
        }
    }
}


//******************* Timer interaction *******************
void configLibTIM(TIM_HandleTypeDef* t1, U16 t1_freq,
                  TIM_HandleTypeDef* t2, U16 t2_freq,
                  TIM_HandleTypeDef* t3, U16 t3_freq, U16 psc)
{
    adc1_timer = t1;
    adc2_timer = t2;
    adc3_timer = t3;
    configTimer(adc1_timer, psc, t1_freq);
    configTimer(adc2_timer, psc, t2_freq);
    configTimer(adc3_timer, psc, t3_freq);
}


void configTimer(TIM_HandleTypeDef* timer, U16 psc,  U16 timer_int_freq_hz) {
    __HAL_TIM_DISABLE(timer);
    __HAL_TIM_SET_COUNTER(timer, 0);
    // Maybe look at this?
    U32 reload;
    do {
        reload = (TIM_CLOCK_BASE_FREQ/psc) / timer_int_freq_hz;
        psc *= 2;
    } while (reload > TIM_MAX_VAL);

    psc /= 2;
    __HAL_TIM_SET_PRESCALER(timer, psc);
    __HAL_TIM_SET_AUTORELOAD(timer, reload);
    __HAL_TIM_ENABLE(timer);
}


void startTimers (void) {
    HAL_TIM_Base_Start_IT(adc1_timer);
    HAL_TIM_Base_Start_IT(adc2_timer);
    HAL_TIM_Base_Start_IT(adc3_timer);
}


void stopTimers (void) {
    HAL_TIM_Base_Stop_IT(adc1_timer);
    HAL_TIM_Base_Stop_IT(adc2_timer);
    HAL_TIM_Base_Stop_IT(adc3_timer);
    __HAL_TIM_SET_COUNTER(adc1_timer, 0);
    __HAL_TIM_SET_COUNTER(adc2_timer, 0);
    __HAL_TIM_SET_COUNTER(adc3_timer, 0);
}


// Call this inside the period elapsed callback
void DAQ_TimerCallback (TIM_HandleTypeDef* timer) {
    HAL_TIM_Base_Stop_IT(timer);

    if (timer == adc1_timer && NUM_ADC1_PARAMS > 0) {
        HAL_ADC_Start_DMA(adc1, (uint32_t*)adc1_sample_buffer, NUM_ADC1_PARAMS);
    }
    else if (timer == adc2_timer && NUM_ADC2_PARAMS > 0) {
        HAL_ADC_Start_DMA(adc2, (uint32_t*)adc2_sample_buffer, NUM_ADC2_PARAMS);
    }
    else if (timer == adc3_timer && NUM_ADC3_PARAMS > 0) {
        HAL_ADC_Start_DMA(adc3, (uint32_t*)adc3_sample_buffer, NUM_ADC3_PARAMS);
    }
}




//******************* CAN Handling *******************

// Redesign option: pull this into a queue and handle not in an ISR
void sensor_can_message_handle (CAN_HandleTypeDef* hcan, U32 rx_mailbox)
{
    CAN_RxHeaderTypeDef rx_header;
    CAN_MSG message;

    // Get the message
    if (HAL_CAN_GetRxMessage(hcan, rx_mailbox, &rx_header, message.data) != HAL_OK) {
        // Handle errors ?
        return;
    }
    message.rtr_bit = rx_header.RTR;
    message.id = rx_header.ExtId;
    message.dlc = rx_header.DLC;

    // Check the CAN params for a match
    for (U8 i = 0; i < NUM_CAN_SENSOR_PARAMS; i++) {

        CAN_SENSOR_PARAM* param = &can_sensor_params[i];
        CAN_SENSOR sensor = param->can_sensor;
        SENSOR_CAN_MESSAGE can_info = sensor.messages[param->message_idx];

        // check for ID match between this param message id and the message id
        if (can_info.message_id == message.id) {
            U16 data = 0;
            U8 shift = 0;
            // Get correct data based on byte order
            if (sensor.byte_order == LSB) {

                for (U8 b = can_info.data_start; b <= can_info.data_end; b++) {
                    data &= message.data[b] << shift;
                    shift += 8;
                }
            }

            else if (sensor.byte_order == MSB) {
                for (U8 b = can_info.data_end; b >= can_info.data_start; b--) {
                    data &= message.data[b] << shift;
                    shift += 8;
                }
            }
            // Add the data to the buffer
            add_to_buffer(&param->buffer, data);

        }
    }
}




//******************* Buffer interaction *******************

// Note: Semaphore probably not needed for buffer interaction because reset is atomic

S8 buffer_full (U16_BUFFER* buffer) {
    if (buffer == NULL) {
        return BUFFER_ERR;
    }
    return buffer->fill_level == buffer->buffer_size;
}

S8 add_to_buffer (U16_BUFFER* buffer, U16 toadd) {
    if (buffer == NULL) {
        return BUFFER_ERR;
    }

    if (buffer_full(buffer)) {
        return BUFFER_ERR;
    }

    buffer->buffer[buffer->fill_level] = toadd;
    buffer->fill_level++;
    return BUFFER_SUCCESS;
}

S8 reset_buffer (U16_BUFFER* buffer) {
    if (buffer == NULL) {
        return BUFFER_ERR;
    }

    buffer->fill_level = 0;
    return BUFFER_SUCCESS;
}

// Could average up to the fill level, returns error for now
S8 average_buffer (U16_BUFFER* buffer, U16* avg) {
    if (buffer == NULL || !buffer_full(buffer)) {
        return BUFFER_ERR;
    }
    U16 calc_avg = 0;
    for (U16 i = 0; i < buffer->buffer_size; i++) {
        calc_avg += buffer->buffer[i];
    }

    *avg = calc_avg / buffer->buffer_size;
    return BUFFER_SUCCESS;

}


S8 apply_can_sensor_conversion (CAN_SENSOR* sensor, U8 msg_idx, float data_in, float* data_out) {
	if (!sensor || !data_out) return CONV_ERR;

	DATA_SCALAR scalar = sensor->messages[msg_idx].output.scalar;
	*data_out = (data_in + scalar.offset) * scalar.quantization; // Verify this is the case for all sensors
    return CONV_SUCCESS;
}



S8 apply_analog_sensor_conversion (ANALOG_SENSOR* sensor, float data_in, float* data_out) {
	if (!sensor || !data_out) return CONV_ERR;

	if (sensor->model.type == SPECIAL) {
		// take care of any fucky cases
		return apply_special_conversions(sensor, data_in, data_out);
    }
    switch (sensor->model.measurement_unit) {
		case VOLTS:
			return convert_voltage_load(sensor, data_in, data_out);
		case OHMS:
			return convert_resistive_load(sensor, data_in, data_out);
		case MILLIAMPS:
			return convert_current_load(sensor, data_in, data_out);
		default:
			// apply no conversion
			*data_out = data_in;

    }


    return CONV_ERR;
}

// gets the first possible voltage divider scalar
float get_voltage_div1_scalar(ANALOG_SENSOR* sensor) {
	OUTPUT_MODEL model = sensor->model;
	if (model.rin == 0) {// float comparison to 0 should be ok....
		return 1;
	}

	if (model.rdown == 0) {
		// this config sinks current straight to ground???
		return 0;
	}

	return model.rdown/ (model.rin + model.rdown);
}



// gets the
float get_voltage_div2_scalar(ANALOG_SENSOR* sensor) {
	OUTPUT_MODEL model = sensor->model;
	if (model.rfilt == 0) {// float comparison to 0 should be ok....
		return 1;
	}

	if (model.rdiv == 0) {
		// this config sinks current straight to ground???
		return 0;
	}

	return model.rdiv / (model.rfilt + model.rdiv);
}


inline float adc_to_volts(U16 adc_reading, U8 resolution_bits) {

	return adc_reading >= ADC_VOLTAGE ? ADC_VOLTAGE : (adc_reading * ADC_VOLTAGE) / (1 << resolution_bits);
}


S8 convert_voltage_load (ANALOG_SENSOR* sensor, float data_in, float* data_out) {

	OUTPUT_MODEL model = sensor->model;

	if (model.r3v != 0 || model.r5v != 0) {
		// pullups engaged = bad
		*data_out = data_in;
		return CONV_ERR;
	}

	// calculate the voltage sourced by the sensor
	float v_read = adc_to_volts(data_in, sensor->output.data_size_bits);
	float div1 = get_voltage_div1_scalar(sensor);
	float div2 = get_voltage_div2_scalar(sensor);

	// configuration error i think
	if (div1 == 0 || div2 == 0) {
		*data_out = data_in;
		return CONV_ERR;
	}


	float v_sensor = v_read / (div1 * div2);

// now convert voltage to useful units according to the model
	switch (model.type) {
		case RATIOMETRIC_LINEAR:// uses a percentage of supply
		{
			float x0 = model.supply_voltage * model.low_bar/100;
			float y0 = model.low_bar_value;
			float x1 = model.supply_voltage * model.high_bar/100;
			float y1 = model.high_bar_value;

			if (v_sensor < x0) {//use low bar as percent if ratiometric
				//Might indicate a sensor error....
				//handle_DAM_error(SENSOR_ERR);
				*data_out = y0;
			}
			else if (v_sensor > x1) {
				*data_out = y1;
			}
			else {//interpolate
				*data_out = interpolate(x0, y0, x1, y1 ,v_sensor);
			}

			return CONV_SUCCESS;
		}


		case ABSOLUTE_LINEAR:
		{
			float x0 = model.low_bar;
			float y0 = model.low_bar_value;
			float x1 = model.high_bar;
			float y1 = model.high_bar_value;

			if (v_sensor < x0) {
				*data_out = y1;
			}
			else if (v_sensor > x1) {
				*data_out = model.high_bar_value;
			}
			else {//interpolate
				*data_out = interpolate(x0, y0, x1, y1 ,v_sensor);
			}

			return CONV_SUCCESS;
		}

		case TABULAR:
		{
			if (sensor->model.table == NULL) { // config error
				*data_out = data_in;
				return CONV_ERR;
			}

			return interpolate_table_linear(sensor->model.table, v_sensor, data_out);
		}
		default:
		{
			*data_out = data_in;
			return CONV_ERR;
		}


	}

}





S8 convert_resistive_load (ANALOG_SENSOR* sensor, float data_in, float* data_out) {
	OUTPUT_MODEL model = sensor->model;

	if (model.r3v == 0 && model.r5v == 0) {
		// pullups NOT engaged = bad
		*data_out = data_in;
		return CONV_ERR;
	}
	if (model.r3v != 0 && model.r5v != 0) {
		// both pullups engaged is also bad
		*data_out = data_in;
		return CONV_ERR;
	}

	float v_read = adc_to_volts(data_in, sensor->output.data_size_bits);
	float div1 = get_voltage_div1_scalar(sensor);
	float div2 = get_voltage_div2_scalar(sensor);

	float r2 = model.r3v != 0 ? model.r3v : model.r5v; //get the other resistor
	float v_sensor = v_read / div2;

	// config error
	if (div1 != 1 || div2 == 0) {
		*data_out = data_in;
		return CONV_ERR;
	}

	// using supply_voltage here is assumming config correct
	// scale =  R_sense/ (r2 + R_sense),
	// supply_voltage * R_sense/(r2 + r_sense)= vsensor
	float r_sensor = ((v_sensor / model.supply_voltage) * r2) / ((v_sensor / model.supply_voltage) - 1);

	switch (model.type) {
		case RATIOMETRIC_LINEAR:
		case ABSOLUTE_LINEAR: // maybe these are different???
		{
			float x0 = model.low_bar;
			float y0 = model.low_bar_value;
			float x1 = model.high_bar;
			float y1 = model.high_bar_value;

			if (r_sensor < x0) {
				*data_out = y0;
			}
			else if (r_sensor > x1) {
				*data_out = y1;
			}
			else {//interpolate
				*data_out = interpolate(x0, y0, x1, y1 ,r_sensor);
			}

			return CONV_SUCCESS;
		}
		case TABULAR:
		{
			return interpolate_table_linear(sensor->model.table, r_sensor, data_out);
		}
		default:
		{
			*data_out = data_in;
			return CONV_ERR;
		}

	}
}


S8 convert_current_load (ANALOG_SENSOR* sensor, float data_in, float* data_out) {
	OUTPUT_MODEL model = sensor->model;

	if (model.r3v != 0 || model.r5v != 0) {
		// pullups engaged = bad
		*data_out = data_in;
		return CONV_ERR;
	}

	// calculate the voltage sourced by the sensor
	float v_read = adc_to_volts(data_in, sensor->output.data_size_bits);
	float div1 = get_voltage_div1_scalar(sensor);
	float div2 = get_voltage_div2_scalar(sensor);

	// rin should be 0, using rdn as shunt
	if (div1 != 1 || div2 == 0) {
		*data_out = data_in;
		return CONV_ERR;
	}

	float v_sensor = v_read / div2;
	//now convert voltage to mA using V/R = R
	float ma_sensor = (v_sensor/model.rdown)  * 1000;

	switch (model.type) {
		case RATIOMETRIC_LINEAR:// uses a percentage of supply
		case ABSOLUTE_LINEAR: // Drop through for now
		{

			float x0 = model.low_bar;
			float y0 = model.low_bar_value;
			float x1 = model.high_bar;
			float y1 = model.high_bar_value;

			if (ma_sensor < x0) {
				*data_out = y0;
			}
			else if (ma_sensor > x1) {
				*data_out = y1;
			}
			else {//interpolate
				*data_out = interpolate(x0, y0, x1, y1 , ma_sensor);
			}

			return CONV_SUCCESS;
		}
		case TABULAR:
		{
			// does this exist?
			return interpolate_table_linear(sensor->model.table, ma_sensor, data_out);
		}
		default:
		{
			*data_out = data_in;
			return CONV_ERR;
		}
	}

}



inline float interpolate(float x0, float y0, float x1, float y1, float x) {
	return ((y0 * (x1 - x)) + (y1 * (x-x0))) / (x1-x0);
}

S8 interpolate_table_linear (TABLE* table, float data_in, float* data_out) {
	U16 entries = table->num_entries;
	if (!table || !entries) {
		*data_out = data_in;
		return CONV_ERR;
	}

	if (data_in < table->dependent_vars[0]) {
		// if off bottom edge return bottom val
		*data_out = table->independent_vars[0];
		return CONV_SUCCESS;
	}

	if (data_in < table->dependent_vars[entries-1]) {
		// if off top edge return top val
		*data_out = table->independent_vars[entries-1];
		return CONV_SUCCESS;
	}


	for (U16 i = 0; i < entries-1; i++) {
		float x0 = table->dependent_vars[i];
		float y0 = table->independent_vars[i];
		float x1 = table->dependent_vars[i+1];
		float y1 = table->independent_vars[i+1];

		if (data_in >= x0 && data_in <= x1) {
			*data_out = interpolate(x0, y0, x1, y1, data_in);
			return CONV_SUCCESS;
		}

	}

	*data_out = data_in;
	return CONV_ERR;
}




S8 apply_special_conversions (ANALOG_SENSOR* sensor, float data_in, float* data_out) {
	*data_out = data_in;
	return CONV_SUCCESS;
}



S8 apply_filter (U16_BUFFER* buffer, FILTERED_PARAM* filter)
{
    // TODO - implement filtering
    return BUFFER_SUCCESS;
}





