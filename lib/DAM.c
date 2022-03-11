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
TIM_HandleTypeDef* tim11_ptr;
TIM_HandleTypeDef* tim14_ptr;

#define TIMER_PSC 16
#define ADC1_SCHEDULING_FREQUENCY_HZ 1000
#define ADC2_SCHEDULING_FREQUENCY_HZ 1000
#define ADC3_SCHEDULING_FREQUENCY_HZ 1000


#define BUCKET_SEND_TASK_STACK_SIZE 128
#define BUCKET_TASK_NAME_BASE "send_bucket_task_"
#define BUCKET_SEND_MAX_ATTEMPTS 5
#define INITIAL_DATA 0xAAf
#define DATA_CONV_FAILURE_REPLACEMENT -1

#define INIT_TX_DELAY_TIME_ms 10

static DAM_ERROR_STATE latched_error_state = NO_ERRORS;
static boolean hasInitialized = FALSE;
static GPIO_TypeDef* status_led_port;
static U16 status_led_pin;


// set the LED state of
void change_led_state(U8 sender, void* parameter, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
    // this function will set the LED to high or low, depending on remote_param
    // the LED to change is dependent on the parameter stored on this module (*((U16*)parameter))
	HAL_GPIO_WritePin(status_led_port, status_led_pin, !!remote_param);
}


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


// TODO really good docs on this function
void DAM_init(CAN_HandleTypeDef* gcan, U8 this_module_id, CAN_HandleTypeDef* scan,
			  ADC_HandleTypeDef* adc1, ADC_HandleTypeDef* adc2, ADC_HandleTypeDef* adc3,
			  TIM_HandleTypeDef* tim10, TIM_HandleTypeDef* tim11, TIM_HandleTypeDef* tim14,
			  GPIO_TypeDef* stat_led_GPIOx, U16 stat_led_Pin)
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
    	tim11_ptr = tim11;
    	tim14_ptr = tim14;
    	status_led_port = stat_led_GPIOx;
    	status_led_pin = stat_led_Pin;

        // Run once initialization code
    	// TODO should this be here or somewhere else?
        if (init_can(gcan_ptr, this_module_id, BXTYPE_MASTER))
        {
            handle_DAM_error(INITIALIZATION_ERROR);
        }
        set_all_params_state(TRUE);

        // CAN commands for the communication with the DLM
        add_custom_can_func(SET_LED_STATE, &change_led_state, TRUE, NULL);
        add_custom_can_func(SEND_BUCKET_PARAMS, &send_bucket_params, TRUE, NULL);
        add_custom_can_func(BUCKET_OK, &bucket_ok, TRUE, NULL);
        add_custom_can_func(REQUEST_BUCKET, &bucket_requested, TRUE, NULL);

        configLibADC(adc1_ptr, adc2_ptr, adc3_ptr);
        configLibTIM(tim10_ptr, ADC1_SCHEDULING_FREQUENCY_HZ,
                     tim11_ptr, ADC2_SCHEDULING_FREQUENCY_HZ,
                     tim14_ptr, ADC3_SCHEDULING_FREQUENCY_HZ, TIMER_PSC);
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
	// All code needed for DLM-DAM reset goes here
	stopDataAq();

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
			if (xTaskCreate(send_bucket_task, name_buf, BUCKET_SEND_TASK_STACK_SIZE,
							(void*) bucket, osPriorityNormal, NULL) != pdPASS)
			{
				// set error state
				handle_DAM_error(INITIALIZATION_ERROR);
			}
		}

		bucket++;
	}

	// start collecting data!
	startDataAq();
	hasInitialized = TRUE;
}


/* DAM_main_task
 * Main task state machine
 */
void DAM_main_task (void)
{
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
			if (all_buckets_ok && latched_error_state == NO_ERRORS)
			{
				if ((HAL_GetTick() - last_timer) >= 500)
				{
					HAL_GPIO_TogglePin(status_led_port, status_led_pin);
					last_timer = HAL_GetTick();
				}
			}
    	}
    }
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
	U32 avg;
	float data_in;
	float converted_data;

	CAN_SENSOR_PARAM* param = can_sensor_params;
    while (param - can_sensor_params < NUM_CAN_SENSOR_PARAMS)
    {
		if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS)
		{
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
							 GET_U16_MSB(bucket->frequency),
							 GET_U16_LSB(bucket->frequency), 0);

    		osDelay(INIT_TX_DELAY_TIME_ms); // Delay to avoid flooding the TX_queue
    		break;

    	case BUCKET_GETTING_DATA:
    		// yield the sending task when not requested
    		taskYIELD();
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

    // this should never be reached
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
    if (bucket != NULL && bucket->state <= BUCKET_CONFIG_SENDING_FRQ )
    {
    	// dont reset the state of bucket getting data/sending
        bucket->state = BUCKET_CONFIG_SENDING_FRQ;
    }
    else
    {
        handle_DAM_error(BUCKET_NOT_RECOGNIZED);
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
    	// TODO delete this
    	u16_tester.data++;

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



