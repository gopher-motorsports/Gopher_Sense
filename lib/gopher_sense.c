// gopher_sense.c
//  The main file for the Gopher Sense library. This file will handle all of
//  the data when it comes in from the data buffers and sending the data over
//  GopherCAN to the data logger

#include "gopher_sense.h"
#include "gsense_structs.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "main.h"
#include "module_hw_config.h"

#if NEED_ADC
ADC_HandleTypeDef* adc1_ptr;
ADC_HandleTypeDef* adc2_ptr;
ADC_HandleTypeDef* adc3_ptr;
#endif

CAN_HandleTypeDef* gcan_ptr;

#if NEED_HW_TIMER
TIM_HandleTypeDef* tim10_ptr;
#endif

#define TIMER_PSC 16
#define ADC_READING_FREQUENCY_HZ 1000

#define TASK_STACK_SIZE 512
#define PARAM_SEND_MAX_ATTEMPTS 5
#define INITIAL_DATA 0.0f
#define DATA_CONV_FAILURE_REPLACEMENT -1

#define INIT_TX_DELAY_TIME_ms 10
#define NO_CONNECTION_TIMEOUT_ms 2000

#define MAX_TIME_BETWEEN_TX_ms 2500

static GSENSE_ERROR_STATE latched_error_state = NO_ERRORS;
static boolean hasInitialized = FALSE;
static GPIO_TypeDef* status_led_port;
static U16 status_led_pin;
static U32 last_dlm_heartbeat = 0;


// static function declarations
static void ADC_sensor_service(void);
#if NEED_ADC > 0
static void service_ADC(ANALOG_SENSOR_PARAM* adc_params, U32 num_params);
#endif
static void handle_param_sending(void);
static void handle_gsense_led(void);
static void handle_gsense_error(GSENSE_ERROR_STATE error_state);
static GENERAL_PARAMETER* find_parameter_from_GCAN(CAN_INFO_STRUCT* can_param);
static S8 fill_gcan_param_data(CAN_INFO_STRUCT* can_param, float data);

// gsense_init
//  This function will init the Gopher Sense library. This includes setting up
//  the timer for ADC buffer transfers and the main task to control the library
//  and LED
// params:
//  CAN_HandleTypeDef* gcan:      Pointer to the CAN handle being used for gopherCAN on
//								  this module. GopherCAN init should be called before this
//								  function
//  ADC_HandleTypeDef* adc1:      Pointer to ADC1 handle. This can be NULL, but make sure it
//								  is configured that way in the yaml
//  ADC_HandleTypeDef* adc2:	  Pointer to ADC2 handle. This can be NULL, but make sure it
//								  is configured that way in the yaml
//  ADC_HandleTypeDef* adc3:	  Pointer to ADC3 handle. This can be NULL, but make sure it
//								  is configured that way in the yaml
//  TIM_HandleTypeDef* tim10:     The timer that will be used for the timer interrupt. Make sure
//								  interrupts are enabled and the library callback is defined in main
//  GPIO_TypeDef* stat_led_GPIOx: Port for the LED for the library
//  U16 stat_led_Pin:			  Pin for the LED for the library
// returns:
//  NO_ERRORS on ok init, INITIALIZATION_ERROR on bad init
#if NEED_ADC
GSENSE_ERROR_STATE gsense_init(CAN_HandleTypeDef* gcan, ADC_HandleTypeDef* adc1,
						    ADC_HandleTypeDef* adc2, ADC_HandleTypeDef* adc3,
						    TIM_HandleTypeDef* tim10, GPIO_TypeDef* stat_led_GPIOx,
							U16 stat_led_Pin)
#else
GSENSE_ERROR_STATE gsense_init(CAN_HandleTypeDef* gcan,
		                       GPIO_TypeDef* stat_led_GPIOx,
							   U16 stat_led_Pin)
#endif
{

    if (!hasInitialized)
    {
    	// assign all of the pointers
    	gcan_ptr = gcan;
#if NEED_ADC
    	adc1_ptr = adc1;
    	adc2_ptr = adc2;
    	adc3_ptr = adc3;
#endif
#if NEED_HW_TIMER
    	tim10_ptr = tim10;
#endif
    	status_led_port = stat_led_GPIOx;
    	status_led_pin = stat_led_Pin;

    	// we need the LED to do anything
    	if (!stat_led_GPIOx) return INITIALIZATION_ERROR;

    	// create the main task. This wont start until things are initialized
    	// but the LED will be run
    	char name_buf[] = "gsense_main_task";
    	if (xTaskCreate(gsense_main_task, name_buf, TASK_STACK_SIZE, NULL, osPriorityNormal,
    			NULL) != pdPASS)
    	{
    		// we cant blink the LED without the task, so just return
    		return INITIALIZATION_ERROR;
    	}

        // make sure a gcan peripheral was passed in and enable all parameters
    	if (!gcan_ptr)
    	{
    		handle_gsense_error(INITIALIZATION_ERROR);
    		return INITIALIZATION_ERROR;
    	}
        set_all_params_state(TRUE);

    	// check to make sure if there are params in the ADCs or SCAN the correct
    	// handles were passed in
#if NEED_HW_TIMER
    	if (!tim10)
    	{
    		handle_gsense_error(INITIALIZATION_ERROR);
    		return INITIALIZATION_ERROR;
    	}
#endif
#if NUM_ADC1_PARAMS > 0
    	if (!adc1)
		{
    		handle_gsense_error(INITIALIZATION_ERROR);
    		return INITIALIZATION_ERROR;
		}
#endif
#if NUM_ADC2_PARAMS > 0
    	if (!adc2)
		{
    		handle_gsense_error(INITIALIZATION_ERROR);
    		return INITIALIZATION_ERROR;
		}
#endif
#if NUM_ADC3_PARAMS > 0
    	if (!adc3)
		{
    		handle_gsense_error(INITIALIZATION_ERROR);
    		return INITIALIZATION_ERROR;
		}
#endif
#if NEED_ADC
        if (configLibADC(adc1_ptr, adc2_ptr, adc3_ptr))
		{
        	handle_gsense_error(INITIALIZATION_ERROR);
        	return INITIALIZATION_ERROR;
		}
        if (configLibTIM(tim10_ptr, ADC_READING_FREQUENCY_HZ, TIMER_PSC))
        {
        	handle_gsense_error(INITIALIZATION_ERROR);
        	return INITIALIZATION_ERROR;
        }
#endif
    }

    gsense_reset();
    return NO_ERRORS;
}


// lock_param_sending
//  Call this function with a GCAN param to lock this parameter to always send
//  regardless of if there is a change in the value or not
// params:
//  CAN_INFO_STRUCT* can_param: what parameter should always be sent
// returns:
//  0 on success, -1 on parameter not found
S8 lock_param_sending(CAN_INFO_STRUCT* can_param)
{
	GENERAL_PARAMETER* param;
	param = find_parameter_from_GCAN(can_param);
	if (!param) return -1;

	param->status = LOCKED_SEND;
	return 0;
}


// update_and_queue_param_float
//  Add data to the correct gcan variable and set the parameter to SEND_NEEDED
//  if the data is different
S8 update_and_queue_param_float(FLOAT_CAN_STRUCT* can_param, float f)
{
	GENERAL_PARAMETER* param;

	if (can_param->data == f)
	{
		// we dont need to do anything as the data was not changed
		return 0;
	}
	can_param->data = f;

	param = find_parameter_from_GCAN((CAN_INFO_STRUCT*)can_param);
	if (!param) return -1;

	// note that we now need to send the updated value for this parameter
	param->status = SEND_NEEDED;

	return 0;
}


// update_and_queue_param_u32
//  Add data to the correct gcan variable and set the parameter to SEND_NEEDED
//  if the data is different
S8 update_and_queue_param_u32(U32_CAN_STRUCT* can_param, U32 u32)
{
	GENERAL_PARAMETER* param;

	if (can_param->data == u32)
	{
		// we dont need to do anything as the data was not changed
		return 0;
	}
	can_param->data = u32;

	param = find_parameter_from_GCAN((CAN_INFO_STRUCT*)can_param);
	if (!param) return -1;

	// note that we now need to send the updated value for this parameter
	param->status = SEND_NEEDED;

	return 0;
}


// update_and_queue_param_float
//  Add data to the correct gcan variable and set the parameter to SEND_NEEDED
//  if the data is different
S8 update_and_queue_param_u8(U8_CAN_STRUCT* can_param, U8 u8)
{
	GENERAL_PARAMETER* param;

	if (can_param->data == u8)
	{
		// we dont need to do anything as the data was not changed
		return 0;
	}
	can_param->data = u8;

	param = find_parameter_from_GCAN((CAN_INFO_STRUCT*)can_param);
	if (!param) return -1;

	// note that we now need to send the updated value for this parameter
	param->status = SEND_NEEDED;

	return 0;
}


// gsense_reset
//  Reset all of the sensor buffers and enable sending of all of the parameters
void gsense_reset(void)
{
	// Reset all of the buffers. Only do the ones that exist
#if NUM_ADC1_PARAMS > 0
	for (U8 i = 0; i < NUM_ADC1_PARAMS; i++)
	{
		reset_buffer(&adc1_sensor_params[i].buffer);
	}
#endif // NUM_ADC1_PARAMS > 0
#if NUM_ADC2_PARAMS > 0
	for (U8 i = 0; i < NUM_ADC2_PARAMS; i++)
	{
		reset_buffer(&adc2_sensor_params[i].buffer);
	}
#endif // NUM_ADC2_PARAMS > 0
#if NUM_ADC3_PARAMS > 0
	for (U8 i = 0; i < NUM_ADC3_PARAMS; i++)
	{
		reset_buffer(&adc3_sensor_params[i].buffer);
	}
#endif // NUM_ADC3_PARAMS > 0

	// enable all params, set status to NO_SEND_NEEDED
	GENERAL_PARAMETER* param;

	for (param = param_list; param - param_list < NUM_CAN_PARAMS; param++)
	{
		if (param->status != LOCKED_SEND) param->status = NO_SEND_NEEDED;
		param->can_param->update_enabled = TRUE;
		fill_gcan_param_data(param->can_param, INITIAL_DATA); // Set some initial value
	}

	// start collecting data!
	startDataAq();
	hasInitialized = TRUE;
}


// gsense_main_task
//  Main gsense task. This handles servicing the ADCs, sending parameters over
//  CAN, and flashing the LED. This tasks runs at 1000Hz
void gsense_main_task(void* param)
{
	// param is unused
	param = NULL;

    while (1)
    {
    	if (hasInitialized)
    	{
    		ADC_sensor_service();
    		handle_param_sending();
    	}

    	handle_gsense_led();
    	osDelay(1);
    }

    // This should not be reached. Panic
    handle_gsense_error(TASK_EXIT_ERROR);
}


// ADC_sensor_service
//  run though all of the active ADCs and service the data in the buffer, moving
//  the data to the GCAN variable associated with this sensor
static void ADC_sensor_service(void)
{
	// TODO need a mutex here for each ADC

#if NUM_ADC1_PARAMS > 0
	service_ADC(adc1_sensor_params, NUM_ADC1_PARAMS);
#endif // NUM_ADC1_PARAMS > 0

#if NUM_ADC2_PARAMS > 0
	service_ADC(adc2_sensor_params, NUM_ADC2_PARAMS);
#endif // NUM_ADC2_PARAMS > 0

#if NUM_ADC3_PARAMS > 0
	service_ADC(adc3_sensor_params, NUM_ADC3_PARAMS);
#endif // NUM_ADC3_PARAMS > 0
}


// service_ADC
//  function that can be called with any ADC to transfer the data
#if NEED_ADC > 0
static void service_ADC(ANALOG_SENSOR_PARAM* adc_params, U32 num_params)
{
	float converted_data;
	U16 avg;
	ANALOG_SENSOR_PARAM* param;
	GENERAL_PARAMETER* gsense_param;

	// loop through each parameter in this ADC buffer
	for (param = adc_params; param - adc_params < num_params; param++)
	{
		if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS)
		{
			continue;
		}

		// convert the average of the ADC buffer to a float with the real world value
		if (apply_analog_sensor_conversion(param->analog_sensor, avg, &converted_data) != CONV_SUCCESS)
		{
			// show there is an error on the LED but try again for the next one,
			// as some might still work
			handle_gsense_error(CONVERSION_ERROR);
			continue;
		}

		// fill the data and set this parameter to SEND_NEEDED. The last time this GCAN param was
		// updated will be stored in the last_rx section. Only set the status to SEND_NEEDED if
		// fill_gcan_param_data returns 1, meaning the data has changed
		gsense_param = param->gsense_param;
		if (fill_gcan_param_data(gsense_param->can_param, converted_data) &&
				gsense_param->status != LOCKED_SEND)
		{
			gsense_param->status = SEND_NEEDED;
		}
		gsense_param->can_param->last_rx = HAL_GetTick();
	}
}
#endif // NEED_ADC > 0


// handle_param_sending
//  This function will check each param in the param list and see if they need
//  to be sent to the logger based on if the value has changed and enough time
//  has passed since the value was sent
static void handle_param_sending(void)
{
	GENERAL_PARAMETER* gsense_param;
	U32 this_tick = HAL_GetTick();
	U16 err_count = 0;

	// for each parameter in the param_list
	for (gsense_param = param_list;
		 gsense_param - param_list < NUM_CAN_PARAMS;
		 gsense_param++)
	{
		// check if this parameter needs to be sent. If it has been a long time
		// between sending then just send this parameter anyway
		if (gsense_param->status == NO_SEND_NEEDED &&
			this_tick - gsense_param->can_param->last_tx < MAX_TIME_BETWEEN_TX_ms)
		{
			continue;
		}

		// if it has been long enough since parameter has been sent, send the parameter
		if (this_tick - gsense_param->can_param->last_tx >= gsense_param->ms_between_requests)
		{
			// try to send, if it fails it is most likely due to bus saturation, meaning it is fine
			// to return out of the function and wait for the next osTick to try
			// and send parameters again
			while (send_parameter(PRIO_HIGH, DLM_ID, gsense_param->can_param->param_id) != CAN_SUCCESS)
			{
				if (++err_count > PARAM_SEND_MAX_ATTEMPTS)
				{
					handle_gsense_error(GCAN_TX_FAILED);
				}
				return;
			}

			// this parameter has been sent. Update the status. last_tx will change
			// when the parameter is actually sent
			if (gsense_param->status != LOCKED_SEND) gsense_param->status = NO_SEND_NEEDED;
		}
	}
}


// handle_gsense_led
//  to be called every ms by the main task. This will figure out what state
//  we are in and set the blink pattern accordingly
static void handle_gsense_led(void)
{
	static U8 num_led_blinks = 0;
	static U32 last_blink_time = 0;

    // if we are in an error state, use a blink pattern based on the number of
    // the error state enum
    if (latched_error_state != NO_ERRORS)
    {
    	// there is an error active
    	if (!num_led_blinks)
    	{
    		// long delay and reset
    		if (HAL_GetTick() - last_blink_time >= LED_ERROR_OFF_TIME)
    		{
    			HAL_GPIO_WritePin(status_led_port, status_led_pin, RESET);
    			last_blink_time = HAL_GetTick();
    			num_led_blinks = (U8)latched_error_state << 1; // double so there is an on and off for each blink number
    		}
    	}
    	else
    	{
    		if (HAL_GetTick() - last_blink_time >= LED_ERROR_BLINK_TIME)
    		{
    			HAL_GPIO_TogglePin(status_led_port, status_led_pin);
    			last_blink_time = HAL_GetTick();
    			num_led_blinks--;
    		}
    	}

    	return;
    }

    // check if we're getting a heartbeat signal from the logger
    if ((HAL_GetTick() - last_dlm_heartbeat) < NO_CONNECTION_TIMEOUT_ms)
    {
    	// Data is being collected and response are being received, blink the LED
    	if ((HAL_GetTick() - last_blink_time) >= LED_NO_ERROR_BLINK_TIME)
		{
			HAL_GPIO_TogglePin(status_led_port, status_led_pin);
			last_blink_time = HAL_GetTick();
		}
    }
    else
    {
    	// No CAN comms, something is wrong
		if ((HAL_GetTick() - last_blink_time) >= LED_NO_LOGGER_COMMS_BLINK_TIME)
		{
			HAL_GPIO_TogglePin(status_led_port, status_led_pin);
			last_blink_time = HAL_GetTick();
		}
    }
}


// handle_gsense_error
//  This will set an error state in the library, which will set a blink code
//  and stop collecting data
static void handle_gsense_error(GSENSE_ERROR_STATE error_state)
{
	// TODO send some CAN thing that lets the driver know that logging is not
	// working? Possibly also just restart the library
	latched_error_state = error_state;
	if (hasInitialized) stopDataAq();
}


// find_parameter_from_GCAN
//  Pass in a GCAN struct, get the parameter pointer from the param_list. Will
//  return NULL if the matching param is not found
static GENERAL_PARAMETER* find_parameter_from_GCAN(CAN_INFO_STRUCT* can_param)
{
	GENERAL_PARAMETER* param;

	for (param = param_list; param - param_list < NUM_CAN_PARAMS; param++)
	{
		if (param->can_param->param_id == can_param->param_id)
		{
			return param;
		}
	}

	// no param was found that matches
	return NULL;
}


// fill_gcan_param_data
//  Fills in the data from a float using the correct type. Returns 1 if the parameter
//  is changed, and 0 if it is not
static S8 fill_gcan_param_data(CAN_INFO_STRUCT* can_param, float data)
{
	switch (parameter_data_types[can_param->param_id])
	{
	case UNSIGNED8:
		if (((U8_CAN_STRUCT*)(can_param))->data != (U8)data)
		{
			((U8_CAN_STRUCT*)(can_param))->data = (U8)data;
			return TRUE;
		}
		return FALSE;

	case UNSIGNED16:
		if (((U16_CAN_STRUCT*)(can_param))->data != (U16)data)
		{
			((U16_CAN_STRUCT*)(can_param))->data = (U16)data;
			return TRUE;
		}
		return FALSE;

	case UNSIGNED32:
		if (((U32_CAN_STRUCT*)(can_param))->data != (U32)data)
		{
			((U32_CAN_STRUCT*)(can_param))->data = (U32)data;
			return TRUE;
		}
		return FALSE;

	case UNSIGNED64:
		if (((U64_CAN_STRUCT*)(can_param))->data != (U64)data)
		{
			((U64_CAN_STRUCT*)(can_param))->data = (U64)data;
			return TRUE;
		}
		return FALSE;

	case SIGNED8:
		if (((S8_CAN_STRUCT*)(can_param))->data != (S8)data)
		{
			((S8_CAN_STRUCT*)(can_param))->data = (S8)data;
			return TRUE;
		}
		return FALSE;

	case SIGNED16:
		if (((S16_CAN_STRUCT*)(can_param))->data != (S16)data)
		{
			((S16_CAN_STRUCT*)(can_param))->data = (S16)data;
			return TRUE;
		}
		return FALSE;

	case SIGNED32:
		if (((S32_CAN_STRUCT*)(can_param))->data != (S32)data)
		{
			((S32_CAN_STRUCT*)(can_param))->data = (S32)data;
			return TRUE;
		}
		return FALSE;

	case SIGNED64:
		if (((S64_CAN_STRUCT*)(can_param))->data != (S64)data)
		{
			((S64_CAN_STRUCT*)(can_param))->data = (S64)data;
			return TRUE;
		}
		return FALSE;

	case FLOATING:
		if (((FLOAT_CAN_STRUCT*)(can_param))->data != data)
		{
			((FLOAT_CAN_STRUCT*)(can_param))->data = data;
			return TRUE;
		}
		return FALSE;
		break;

	default:
		handle_gsense_error(DATA_ASSIGNMENT_ERROR);
		break;
	}

	return FALSE;
}


//*************** GopherCAN callbacks *****************

// log_complete
//  Handler for the LOG_COMPLETE gopherCAN command
//  DLM is actively logging & communicating
void log_complete(MODULE_ID sender, void* parameter,
                  U8 UNUSED1, U8 UNUSED2, U8 UNUSED3, U8 UNUSED4)
{
	if (sender != DLM_ID) return;

	last_dlm_heartbeat = HAL_GetTick();
}


// End of gopher_sense.c
