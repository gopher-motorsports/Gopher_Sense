/*
 * DAM.c
 *
 *  Created on: Jun 9, 2021
 *      Author: ian
 */


#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "main.h"
#include "stm32f7xx_hal_can.h"
#include "DAM.h"
#include "sensor_hal.h"
#include "dam_hw_config.h"

ADC_HandleTypeDef* adc1_ptr;
ADC_HandleTypeDef* adc2_ptr;
ADC_HandleTypeDef* adc3_ptr;
//DMA_HandleTypeDef hdma_adc1;
//DMA_HandleTypeDef hdma_adc2;
//DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef* gcan_ptr;
CAN_HandleTypeDef* scan_ptr;

TIM_HandleTypeDef* tim10_ptr;

#define TIMER_PSC 16
#define ADC_READING_FREQUENCY_HZ 1000

#define TASK_STACK_SIZE 512
#define BUCKET_TASK_NAME_BASE "send_bucket_task_"
#define BUCKET_SEND_MAX_ATTEMPTS 5
#define INITIAL_DATA 0xAAf
#define DATA_CONV_FAILURE_REPLACEMENT -1

#define INIT_TX_DELAY_TIME_ms 10
#define NO_CONNECTION_TIMEOUT_ms 2000

static DAM_ERROR_STATE latched_error_state = NO_ERRORS;
static boolean hasInitialized = FALSE;
static GPIO_TypeDef* status_led_port;
static U16 status_led_pin;
static U32 last_bucket_req = 0;


// Handle LED states for each error for future VIRs
void handle_DAM_error(DAM_ERROR_STATE error_state)
{
	U8 num_led_blinks = 0;
	U8 c;
    latched_error_state = error_state;

    // set the number of LED blinks using the enums
    num_led_blinks = (U8)error_state;
    stopDataAq();

    while(1)
    {
    	for(c = 0; c < num_led_blinks; c++)
    	{
    		HAL_GPIO_WritePin(status_led_port, status_led_pin, GPIO_PIN_SET);
    		osDelay(100);
    		HAL_GPIO_WritePin(status_led_port, status_led_pin, GPIO_PIN_RESET);
    		osDelay(100);
    	}
    	osDelay(800);
    }
}


// DAM_init
//  This function will init the Gopher Sense library. This includes setting up
//  the timer for ADC buffer transfers, the main task to control the DAM, and the
//  individual bucket tasks.
// params:
//  CAN_HandleTypeDef* gcan:      Pointer to the CAN handle being used for gopherCAN on
//								  this module. GopherCAN init should be called before this
//								  function
//  CAN_HandleTypeDef* scan:      Pointer to the sensorCAN handle. This functon will init
//								  sensorCAN, so nothing else must be done for it. This value
//								  can be passed in as NULL if there is no sensorCAN on this
//								  module, but make sure it is configured that way in the yaml
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
void DAM_init(CAN_HandleTypeDef* gcan, CAN_HandleTypeDef* scan,
			  ADC_HandleTypeDef* adc1, ADC_HandleTypeDef* adc2, ADC_HandleTypeDef* adc3,
			  TIM_HandleTypeDef* tim10, GPIO_TypeDef* stat_led_GPIOx, U16 stat_led_Pin)
{
    if (!hasInitialized)
    {
    	// assign all of the pointers
    	gcan_ptr = gcan;
    	scan_ptr = scan;
    	adc1_ptr = adc1;
    	adc2_ptr = adc2;
    	adc3_ptr = adc3;
    	tim10_ptr = tim10;
    	status_led_port = stat_led_GPIOx;
    	status_led_pin = stat_led_Pin;

    	// check to make sure if there are params in the ADCs or SCAN the correct
    	// handles were passed in
    	if (!stat_led_GPIOx) handle_DAM_error(INITIALIZATION_ERROR);
    	if (!tim10) handle_DAM_error(INITIALIZATION_ERROR);
#if NUM_ADC1_PARAMS > 0
    	if (!adc1) handle_DAM_error(INITIALIZATION_ERROR);
#endif
#if NUM_ADC2_PARAMS > 0
    	if (!adc2) handle_DAM_error(INITIALIZATION_ERROR);
#endif
#if NUM_ADC3_PARAMS > 0
    	if (!adc3) handle_DAM_error(INITIALIZATION_ERROR);
#endif

        // make sure a gcan peripheral was passed in and enable all parameters for DAMs
    	if (!gcan_ptr) handle_DAM_error(INITIALIZATION_ERROR);
        set_all_params_state(TRUE);

#if NUM_CAN_SENSOR_PARAMS > 0
        if (scan_ptr)
        {
        	// TODO init and start the sensorCAN bus
        	// make sure that filters are working correctly
        }
        else
        {
        	// there was not a sensor can passed in when there should be
        	handle_DAM_error(INITIALIZATION_ERROR);
        }
#endif

        // CAN commands for the communication with the DLM
        add_custom_can_func(SEND_BUCKET_PARAMS, &send_bucket_params, TRUE, NULL);
        add_custom_can_func(BUCKET_OK, &bucket_ok, TRUE, NULL);
        add_custom_can_func(REQUEST_BUCKET, &bucket_requested, TRUE, NULL);

        if (configLibADC(adc1_ptr, adc2_ptr, adc3_ptr)) handle_DAM_error(INITIALIZATION_ERROR);
        if (configLibTIM(tim10_ptr, ADC_READING_FREQUENCY_HZ, TIMER_PSC)) handle_DAM_error(INITIALIZATION_ERROR);
    }

    DAM_reset();
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


// DAM_reset
//  Reset all of the sensor buffers and start the DLM-DAM initialization process
//  over again
void DAM_reset(void)
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
#if NUM_CAN_SENSOR_PARAMS > 0
	for (U8 i = 0; i < NUM_CAN_SENSOR_PARAMS; i++)
	{
		reset_buffer(&can_sensor_params[i].buffer);
	}
#endif // NUM_CAN_SENSOR_PARAMS > 0

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
			//create bucket tasks
			char name_buf[30];
			sprintf(name_buf, "%s%d", BUCKET_TASK_NAME_BASE, bucket->bucket_id);
			if (xTaskCreate(send_bucket_task, name_buf, TASK_STACK_SIZE,
							(void*) bucket, osPriorityLow, NULL) != pdPASS)
			{
				// set error state
				handle_DAM_error(INITIALIZATION_ERROR);
			}
		}

		bucket++;
	}

	// create the DAM main task
	char name_buf[] = "DAM_main_task";
	if (xTaskCreate(DAM_main_task, name_buf, TASK_STACK_SIZE, NULL, osPriorityNormal,
			NULL) != pdPASS)
	{
		// set error state
		handle_DAM_error(INITIALIZATION_ERROR);
	}

	// start collecting data!
	startDataAq();
	hasInitialized = TRUE;
}


/* DAM_main_task
 * Main task state machine
 * This should run at a priority above the bucket response tasks as we want
 * the data to be handled by this task first, then the data to be sent to the
 * DLM
 */
void DAM_main_task(void* param)
{
	// param is unused
	param = NULL;

	boolean all_buckets_ok;
	BUCKET* bucket;
	U32 last_timer = 0;
    // Must have started buffer service task and tx task
    while (1)
    {
    	if (!hasInitialized) // waiting for initialization
    	{
    		osDelay(1);
    	}
    	else
    	{
    		ADC_sensor_service();
    		sensorCAN_service();

    		// toggle the LED at 0.5 second intervals if the all of the buckets
    		// are working normally and there are no errors
    		all_buckets_ok = TRUE;
    		bucket = bucket_list;
			while (bucket - bucket_list < NUM_BUCKETS)
			{
				// check if this bucket is not ready
				if(bucket->state <= BUCKET_CONFIG_SENDING_FRQ)
				{
					all_buckets_ok = FALSE;
					break;
				}
				bucket++;
			}
			if (all_buckets_ok && latched_error_state == NO_ERRORS &&
				HAL_GetTick() - last_bucket_req < NO_CONNECTION_TIMEOUT_ms)
			{
				// blink at 500ms intervals when we are logging correctly
				if ((HAL_GetTick() - last_timer) >= 500)
				{
					HAL_GPIO_TogglePin(status_led_port, status_led_pin);
					last_timer = HAL_GetTick();
				}
			}
			else
			{
				// waiting for the DLM to complete the handshake. Blink at 2 sec intervals
				if ((HAL_GetTick() - last_timer) >= 2000)
				{
					HAL_GPIO_TogglePin(status_led_port, status_led_pin);
					last_timer = HAL_GetTick();
				}
			}

			osDelay(1);
    	}
    }

    // This should not be reached. Panic
    handle_DAM_error(TASK_EXIT_ERROR);
}


void ADC_sensor_service (void)
{
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


void service_ADC(ANALOG_SENSOR_PARAM* adc_params, U32 num_params)
{
	float data_in;
	float converted_data;
	U32 avg;
	ANALOG_SENSOR_PARAM* param = adc_params;

	while (param - adc_params < num_params)
	{
		if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS)
		{
			param++;
			continue;
		}
		data_in = avg;

		if (apply_analog_sensor_conversion(param->analog_sensor, data_in, &converted_data) != CONV_SUCCESS)
		{
			handle_DAM_error(CONVERSION_ERROR);
		}

		// fill the data and set this parameter to dirty
		fill_gcan_param_data(param->bucket_param->can_param, converted_data);
		if (param->bucket_param->status != LOCKED_SEND) param->bucket_param->status = DIRTY;
		fill_analog_subparams(param, converted_data);
		param++;
	}
}


void sensorCAN_service (void)
{
#if NUM_CAN_SENSOR_PARAMS > 0
	U32 avg;
	float data_in;
	float converted_data;

	CAN_SENSOR_PARAM* param = can_sensor_params;
    while (param - can_sensor_params < NUM_CAN_SENSOR_PARAMS)
    {
		if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS)
		{
			param++;
			continue;
		}
		data_in = avg;

		if (apply_can_sensor_conversion(param->can_sensor, param->message_idx, data_in, &converted_data) != CONV_SUCCESS)
		{
			handle_DAM_error(CONVERSION_ERROR);
		}

		// fill the data and set this parameter to dirty
		fill_gcan_param_data(param->bucket_param->can_param, converted_data);
		if (param->bucket_param->status != LOCKED_SEND) param->bucket_param->status = DIRTY;
		fill_can_subparams(param, converted_data);
        param++;
    }
#endif
}


// send_bucket_task
//  one task for each bucket. This will handle the interaction with the
//  DLM through init and bucket requests
void send_bucket_task (void* pvParameters)
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

    	case BUCKET_CONFIG_SENDING_FRQ:
    		// Send the frequency until this bucket is requested the first time
    		send_can_command(PRIO_HIGH, DLM_ID, ASSIGN_BUCKET_TO_FRQ,
							 bucket->bucket_id,
							 GET_U16_MSB(bucket->ms_between_req),
							 GET_U16_LSB(bucket->ms_between_req), 0);

    		// wait for twice the time this bucket expects to get the first request in before
    		// sending the frequency again to minimize extra messages
    		osDelay(bucket->ms_between_req << 1);
    		break;

    	case BUCKET_GETTING_DATA:
    		// yield the sending task when not requested
    		osDelay(1);
    		break;

    	case BUCKET_REQUESTED:
    		// send the bucket parameters
			param = bucket->param_list.list;
			while (param - bucket->param_list.list < bucket->param_list.len)
			{
				if (param->status == DIRTY || param->status == LOCKED_SEND)
				{
					U16 err_count = 0;
					while (send_parameter(PRIO_HIGH, DLM_ID, param->can_param->param_id) != CAN_SUCCESS)
					{
						if (++err_count > BUCKET_SEND_MAX_ATTEMPTS)
						{
							handle_DAM_error(GCAN_TX_FAILED);
							break;
						}
						osDelay(1); // Delay due to error
					}

					// set this parameter to clean if it is ok too
					if (param->status != LOCKED_SEND) param->status = CLEAN;
			   }
				param++;
			}

			bucket->state = BUCKET_GETTING_DATA;
			taskYIELD();
    		break;
    	}
    }

    // this should never be reached. Set an error state
    handle_DAM_error(TASK_EXIT_ERROR);
    vTaskDelete(NULL);
}


BUCKET* get_bucket_by_id (U8 bucket_id)
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


void fill_gcan_param_data(CAN_INFO_STRUCT* can_param, float data)
{
	switch (parameter_data_types[can_param->param_id])
	{
	case UNSIGNED8:
		((U8_CAN_STRUCT*)(can_param))->data = (U8)data;
		break;

	case UNSIGNED16:
		((U16_CAN_STRUCT*)(can_param))->data = (U16)data;
		break;

	case UNSIGNED32:
		((U32_CAN_STRUCT*)(can_param))->data = (U32)data;
		break;

	case UNSIGNED64:
		((U64_CAN_STRUCT*)(can_param))->data = (U64)data;
		break;

	case SIGNED8:
		((S8_CAN_STRUCT*)(can_param))->data = (S8)data;
		break;

	case SIGNED16:
		((S16_CAN_STRUCT*)(can_param))->data = (S16)data;
		break;

	case SIGNED32:
		((S32_CAN_STRUCT*)(can_param))->data = (S32)data;
		break;

	case SIGNED64:
		((S64_CAN_STRUCT*)(can_param))->data = (S64)data;
		break;

	case FLOATING:
		((FLOAT_CAN_STRUCT*)(can_param))->data = data;
		break;

	default:
		handle_DAM_error(DATA_ASSIGNMENT_ERROR);
		break;
	}
}


//*************** GopherCAN callbacks *****************
/* send_bucket_params
 * Handler for the SEND_BUCKET_PARAMS gopherCAN command
 * sets each bucket into configuration state
 */
void send_bucket_params (U8 sender, void* param, U8 U1, U8 U2, U8 U3, U8 U4)
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


/* bucket_ok
 * Handler for the BUCKET_OK gopherCAN command
 * sets the passed bucket state to start sending the
 * frequency of this bucket
 */
void bucket_ok(MODULE_ID sender, void* parameter,
               U8 bucket_id, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
    if (sender != DLM_ID) return;

    BUCKET* bucket = get_bucket_by_id(bucket_id);
    if (bucket == NULL)
    {
    	handle_DAM_error(BUCKET_NOT_RECOGNIZED);
    	return;
    }
    if (bucket->state <= BUCKET_CONFIG_SENDING_FRQ )
    {
    	// dont reset the state of bucket getting data/sending
        bucket->state = BUCKET_CONFIG_SENDING_FRQ;
    }

}


/* bucket_requested
 * Handler for the RequestBucket gopherCAN command
 * Sets the state for this bucket to requested, regardless
 * of what state it was in before. The DLM will not request
 * the bucket until it is configured on the DLM, so to make
 * it here everything is ok
 */
void bucket_requested (MODULE_ID sender, void* parameter,
                       U8 bucket_id, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
    BUCKET* bucket = get_bucket_by_id(bucket_id);
    if (bucket == NULL)
    {
        handle_DAM_error(BUCKET_NOT_RECOGNIZED);
        return;
    }

    bucket->state = BUCKET_REQUESTED;

    // note that a new bucket was requested
    last_bucket_req = HAL_GetTick();
}


/* custom_service_can_rx_hardware
 * Definition for the ISR called on CAN message reception
*/
void custom_service_can_rx_hardware(CAN_HandleTypeDef* hcan, U32 rx_mailbox)
{
   if (hcan == gcan_ptr)
   {
       service_can_rx_hardware(hcan, rx_mailbox);
   }

   else if (hcan == scan_ptr)
   {
       sensor_can_message_handle(hcan, rx_mailbox);
   }

   else handle_DAM_error(CAN_HANDLE_NOT_RECOGNIZED); // This case shouldnt happen
}


//*************** GopherCAN tasks *****************
// Service the GopherCAN tx buffer
void gopherCAN_tx_service_task (void)
{
    while(!hasInitialized) osDelay(1); // wait for initialization

    while (1)
    {
        service_can_tx_hardware(gcan_ptr);
        osDelay(1);
    }
}

// Service the GopherCAN rx buffer
void gopherCAN_rx_buffer_service_task (void)
{
    while(!hasInitialized) osDelay(1); // wait for initialization

    while (1)
    {
        if (service_can_rx_buffer())
        {
            handle_DAM_error(RX_BUFFER_HANDLE_ERROR);
        }
        osDelay(1);
    }
}



// This has to do with filtering, so do this whomever implements that
void fill_can_subparams (CAN_SENSOR_PARAM* param, float newdata)
{
    for (U8 i = 0; i < param->num_filtered_subparams; i++)
    {
    	// TODO implement can subparams?
    }
}

// This has to do with filtering, so do this whomever implements that
void fill_analog_subparams (ANALOG_SENSOR_PARAM* param, float newdata)
{
    for (U8 i = 0; i < param->num_filtered_subparams; i++)
    {
    	// TODO implement can subparams?
    }
}


// TODO: add timer interrupt to ensure IDLEtask runs??



