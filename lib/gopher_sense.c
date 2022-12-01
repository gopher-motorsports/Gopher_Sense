// gopher_sense.c
//  The main file for the Gopher Sense library. This file will handle all of
//  the data when it comes in from the data buffers and sending the data over
//  GopherCAN to the data logger

// TODO UPGRADE keyword is used when this is feature that should be changed
// once the logging code is changed as well

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
#define BUCKET_TASK_NAME_BASE "bucket_task_"
#define BUCKET_SEND_MAX_ATTEMPTS 5
#define INITIAL_DATA 0xAAf
#define DATA_CONV_FAILURE_REPLACEMENT -1

#define INIT_TX_DELAY_TIME_ms 10
#define NO_CONNECTION_TIMEOUT_ms 2000

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
static void handle_gsense_error(GSENSE_ERROR_STATE error_state);
static BUCKET* get_bucket_by_id(U8 bucket_id);

// gsense_init
//  This function will init the Gopher Sense library. This includes setting up
//  the timer for ADC buffer transfers, the main task to control the library and
//  LED, and the individual bucket tasks to check if the data is ready to be
//  sent and send it
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
//  when the bucket it is in is requested
// params:
//  CAN_INFO_STRUCT* can_param: what parameter should always be sent
// returns:
//  0 on success, -1 on parameter not found
S8 lock_param_sending(CAN_INFO_STRUCT* can_param)
{
	// run through each parameter in each bucket and check if the CAN_PARAM sent in
	// is the match
	BUCKET* bucket = bucket_list;
	GENERAL_PARAMETER* param;
	while (bucket - bucket_list < NUM_BUCKETS)
	{
		param = bucket->param_list.list;
		while (param - bucket->param_list.list < bucket->param_list.len)
		{
			if (param->can_param->param_id == can_param->param_id)
			{
				// we found the correct parameter in a bucket
				param->status = LOCKED_SEND;
			}
			param++;
		}

		bucket++;
	}

	// parameter not found
	return -1;
}


// update_and_queue_param_float
//  Add data to the correct gcan variable and set the parameter to dirty
//  if the data is different
S8 update_and_queue_param_float(FLOAT_CAN_STRUCT* can_param, float f)
{
	if (can_param->data == f)
	{
		// we dont need to do anything as the data was not changed
		return 0;
	}
	can_param->data = f;

	BUCKET* bucket = bucket_list;
	GENERAL_PARAMETER* param;
	while (bucket - bucket_list < NUM_BUCKETS)
	{
		param = bucket->param_list.list;
		while (param - bucket->param_list.list < bucket->param_list.len)
		{
			if (param->can_param->param_id == can_param->param_id)
			{
				// we found the correct parameter in a bucket
				param->status = DIRTY;
				return 0;
			}
			param++;
		}

		bucket++;
	}

	return -1;
}


// update_and_queue_param_u32
//  Add data to the correct gcan variable and set the parameter to dirty
//  if the data is different
S8 update_and_queue_param_u32(U32_CAN_STRUCT* can_param, U32 u32)
{
	if (can_param->data == u32)
	{
		// we dont need to do anything as the data was not changed
		return 0;
	}
	can_param->data = u32;

	BUCKET* bucket = bucket_list;
	GENERAL_PARAMETER* param;
	while (bucket - bucket_list < NUM_BUCKETS)
	{
		param = bucket->param_list.list;
		while (param - bucket->param_list.list < bucket->param_list.len)
		{
			if (param->can_param->param_id == can_param->param_id)
			{
				// we found the correct parameter in a bucket
				param->status = DIRTY;
				return 0;
			}
			param++;
		}

		bucket++;
	}

	return -1;
}


// update_and_queue_param_float
//  Add data to the correct gcan variable and set the parameter to dirty
//  if the data is different
S8 update_and_queue_param_u8(U8_CAN_STRUCT* can_param, U8 u8)
{
	if (can_param->data == u8)
	{
		// we dont need to do anything as the data was not changed
		return 0;
	}
	can_param->data = u8;

	BUCKET* bucket = bucket_list;
	GENERAL_PARAMETER* param;
	while (bucket - bucket_list < NUM_BUCKETS)
	{
		param = bucket->param_list.list;
		while (param - bucket->param_list.list < bucket->param_list.len)
		{
			if (param->can_param->param_id == can_param->param_id)
			{
				// we found the correct parameter in a bucket
				param->status = DIRTY;
				return 0;
			}
			param++;
		}

		bucket++;
	}

	return -1;
}


// gsense_reset
//  Reset all of the sensor buffers and start the DLM-DAM initialization process
//  over again
// UPGRADE this does not need to be reset with the new data scheme
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

	// enable all bucket params, set status to clean
	BUCKET* bucket = bucket_list;
	GENERAL_PARAMETER* param;
	while (bucket - bucket_list < NUM_BUCKETS)
	{
		bucket->state = BUCKET_CONFIG_INIT;

		param = bucket->param_list.list;
		while (param - bucket->param_list.list < bucket->param_list.len)
		{
			if (param->status != LOCKED_SEND) param->status = CLEAN;
			param->can_param->update_enabled = TRUE;
			fill_gcan_param_data(param->can_param, INITIAL_DATA); // Set some initial value
			param++;
		}
		if (!hasInitialized)
		{
			// create bucket tasks
			// UPGRADE we dont need a ton of these tasks with the new sending scheme
			char name_buf[30];
			sprintf(name_buf, "%s%d", BUCKET_TASK_NAME_BASE, bucket->bucket_id);
			if (xTaskCreate(send_bucket_task, name_buf, TASK_STACK_SIZE,
							(void*) bucket, osPriorityLow, NULL) != pdPASS)
			{
				// set error state but don't return in case the rest work
				handle_gsense_error(INITIALIZATION_ERROR);
			}
		}

		bucket++;
	}

	// start collecting data!
	startDataAq();
	hasInitialized = TRUE;
}


// gsense_main_task
//  Main task state machine. this should run at a priority above the bucket
//  response tasks as we want the data to be handled by this task first, then
//  the data to be sent to the data logger if the data has changed
void gsense_main_task(void* param)
{
	// param is unused
	param = NULL;

    // Must have started buffer service task and tx task
    while (1)
    {
    	if (hasInitialized)
    	{
    		ADC_sensor_service();
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
	ANALOG_SENSOR_PARAM* param = adc_params;

	// loop through each parameter in this ADC buffer
	while (param - adc_params < num_params)
	{
		if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS)
		{
			param++;
			continue;
		}

		// convert the average of the ADC buffer to a float with the real world value
		if (apply_analog_sensor_conversion(param->analog_sensor, avg, &converted_data) != CONV_SUCCESS)
		{
			// show there is an error on the LED but try again for the next one,
			// as some might still work
			handle_gsense_error(CONVERSION_ERROR);
			param++;
			continue;
		}

		// fill the data and set this parameter to dirty. The last time this GCAN param was
		// updated will be stored in the last_rx section. Only set the status to dirty if
		// fill_gcan_param_dirty returns 1, meaning the data has changed
		if (fill_gcan_param_data(param->bucket_param->can_param, converted_data) &&
			param->bucket_param->status != LOCKED_SEND) param->bucket_param->status = DIRTY;
		param->bucket_param->can_param->last_rx = HAL_GetTick();
		param++;
	}
}
#endif // NEED_ADC > 0


// handle_gsense_led
//  to be called every ms by the main task. This will figure out what state
//  we are in and set the blink pattern accordingly
void handle_gsense_led(void)
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

    // check if all the buckets are being filled
    boolean all_buckets_ok = TRUE;
    BUCKET* bucket = bucket_list;
    while (bucket - bucket_list < NUM_BUCKETS)
    {
    	if (bucket->state < BUCKET_GETTING_DATA)
		{
			all_buckets_ok = FALSE;
			break;
		}
		bucket++;
    }

    // check if we're getting a heartbeat signal from the logger
    boolean DLM_active = (HAL_GetTick() - last_dlm_heartbeat) < NO_CONNECTION_TIMEOUT_ms;

    if (all_buckets_ok && DLM_active) {
    	// buckets are collecting data and logger is logging, blink every 500ms
    	if ((HAL_GetTick() - last_blink_time) >= LED_NO_ERROR_BLINK_TIME)
		{
			HAL_GPIO_TogglePin(status_led_port, status_led_pin);
			last_blink_time = HAL_GetTick();
		}
    }
    else {
    	// either a bucket isn't ready yet or DLM isn't logging, blink every 2s
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


// send_bucket_task
//  one task for each bucket. This will handle the interaction with the
//  DLM through init and bucket requests
// UPGRADE this can be considerably gutted with the new data sending scheme
void send_bucket_task(void* pvParameters)
{
    BUCKET* bucket = (BUCKET*) pvParameters;
    GENERAL_PARAMETER* param;

    while(1)
    {
    	switch (bucket->state)
    	{
    	case BUCKET_CONFIG_INIT:
    	case BUCKET_CONFIG_SENDING_PARAMS:
    		// sent the size of this bucket and all the params in it
    		send_can_command(PRIO_HIGH, DLM_ID, SET_BUCKET_SIZE, bucket->bucket_id,
							 (U8)bucket->param_list.len, 0, 0);

			param = bucket->param_list.list;
			while (param - bucket->param_list.list < bucket->param_list.len)
			{
				send_can_command(PRIO_HIGH, DLM_ID, ADD_PARAM_TO_BUCKET, bucket->bucket_id,
								 GET_U16_MSB(param->can_param->param_id),
								 GET_U16_LSB(param->can_param->param_id), 0);
				param++;
			}

			osDelay(INIT_TX_DELAY_TIME_ms); // Delay to avoid flooding the TX_queue
    		break;

    	case BUCKET_GETTING_DATA:
    		// check if this bucket is ready to be sent
    		if (HAL_GetTick() - bucket->last_send >= bucket->ms_between_req)
    		{
    			bucket->state = BUCKET_SENDING;
    		}

    		// yield the sending task when not requested
    		osDelay(1);
    		break;

    	case BUCKET_SENDING:
    		// send the bucket parameters
			param = bucket->param_list.list;
			while (param - bucket->param_list.list < bucket->param_list.len)
			{
				if (param->status == DIRTY || param->status == LOCKED_SEND
						|| HAL_GetTick() - param->last_tx >= MAX_TIME_BETWEEN_TX_ms)
				{
					U16 err_count = 0;
					while (send_parameter(PRIO_HIGH, DLM_ID, param->can_param->param_id) != CAN_SUCCESS)
					{
						if (++err_count > BUCKET_SEND_MAX_ATTEMPTS)
						{
							handle_gsense_error(GCAN_TX_FAILED);
							break;
						}
						osDelay(1); // Delay due to error
					}

					// set this parameter to clean if it is ok too
					if (param->status != LOCKED_SEND) param->status = CLEAN;
					param->last_tx = HAL_GetTick();
			    }
			    param++;
			}

			// flush the TX buffer
			service_can_tx_hardware(gcan_ptr);
			bucket->last_send = HAL_GetTick();

			bucket->state = BUCKET_GETTING_DATA;
			osDelay(1);
    		break;
    	}
    }

    // this should never be reached. Set an error state
    handle_gsense_error(TASK_EXIT_ERROR);
    vTaskDelete(NULL);
}


// get_bucket_by_id
//  used by the GCAN callback function,
static BUCKET* get_bucket_by_id(U8 bucket_id)
{
	BUCKET* ret_bucket = bucket_list;
	while (ret_bucket - bucket_list < NUM_BUCKETS)
    {
        if (ret_bucket->bucket_id == bucket_id)
        {
            return ret_bucket;
        }
        ret_bucket++;
    }
    return NULL;
}


// fill_gcan_param_data
//  Fills in the data from a float using the correct type. Returns 1 if the parameter
//  is changed, and 0 if it is not
S8 fill_gcan_param_data(CAN_INFO_STRUCT* can_param, float data)
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
// send_bucket_params
//  Handler for the SEND_BUCKET_PARAMS gopherCAN command
//  sets each bucket into configuration state
void send_bucket_params(U8 sender, void* param, U8 U1, U8 U2, U8 U3, U8 U4)
{
    if (sender != DLM_ID) return;

    // set the state of all buckets to BUCKET_CONFIG_SENDING_PARAMS
    BUCKET* bucket = bucket_list;
	while (bucket - bucket_list < NUM_BUCKETS)
	{
		bucket->state = BUCKET_CONFIG_SENDING_PARAMS;
		bucket++;
	}
}


// bucket_ok
//  Handler for the BUCKET_OK gopherCAN command
//  sets the passed bucket state to start sending the
//  frequency of this bucket
void bucket_ok(MODULE_ID sender, void* parameter,
               U8 bucket_id, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
    if (sender != DLM_ID) return;

    BUCKET* bucket = get_bucket_by_id(bucket_id);
    if (bucket == NULL)
    {
    	handle_gsense_error(BUCKET_NOT_RECOGNIZED);
    	return;
    }
    if (bucket->state < BUCKET_GETTING_DATA)
    {
    	// if this bucket isn't already getting/sending data, start
    	bucket->state = BUCKET_GETTING_DATA;
    	bucket->last_send = HAL_GetTick();
    }
}

// log_complete
//  Handler for the LOG_COMPLETE gopherCAN command
//  DLM is actively logging & communicating
void log_complete(MODULE_ID sender, void* parameter,
                  U8 UNUSED1, U8 UNUSED2, U8 UNUSED3, U8 UNUSED4)
{
	if (sender != DLM_ID) return;

	last_dlm_heartbeat = HAL_GetTick();
}

//*************** GopherCAN tasks *****************
// Service the GopherCAN tx buffer
void gopherCAN_tx_service_task(void)
{
    while(!hasInitialized) osDelay(1); // wait for initialization

    while (1)
    {
        service_can_tx_hardware(gcan_ptr);
        osDelay(1);
    }
}

// Service the GopherCAN rx buffer
void gopherCAN_rx_buffer_service_task(void)
{
    while(!hasInitialized) osDelay(1); // wait for initialization

    while (1)
    {
        if (service_can_rx_buffer())
        {
            handle_gsense_error(RX_BUFFER_HANDLE_ERROR);
        }
        osDelay(1);
    }
}


