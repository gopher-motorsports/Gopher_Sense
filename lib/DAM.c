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

// TODO Move this to a config file. Maybe auto generate? Maybe just fix at 10k
#define TIMER_PSC 16
#define ADC1_SCHEDULING_FREQUENCY_HZ 100
#define ADC2_SCHEDULING_FREQUENCY_HZ 100
#define ADC3_SCHEDULING_FREQUENCY_HZ 100


#define BUCKET_SEND_TASK_STACK_SIZE 128
#define BUCKET_TASK_NAME_BASE "send_bucket_task_"
#define BUCKET_SEND_MAX_ATTEMPTS 5
#define INITIAL_DATA 0xAA
#define DATA_CONV_FAILURE_REPLACEMENT -1

#define INIT_TX_DELAY_TIME_ms 10

static U64 error_count;
static DAM_ERROR_STATE latched_error_state = NO_ERRORS;
static boolean hasInitialized = FALSE;


void change_led_state(U8 sender, void* parameter, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
    // this function will set the LED to high or low, depending on remote_param
    // the LED to change is dependent on the parameter stored on this module (*((U16*)parameter))
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, !!remote_param);
}


// TODO: Error state behavior. Possibly give control of this to outsize the library
void handle_DAM_error (DAM_ERROR_STATE error_state)
{
    latched_error_state = error_state;
    switch (error_state)
    {
        // log the error?
        case INITIALIZATION_ERROR:
        {
        	// DEBUG
        	while (1);
            NVIC_SystemReset();
            break;
        }
        case CRITICAL_ERROR:
        {
        	// DEBUG
        	while (1);
            NVIC_SystemReset();
            break;
        }
        case CONVERSION_ERROR:
        {
        	// DEBUG
        	while (1);

        	//send can error???
        	break;
        }
        default:
        {
        	// DEBUG
        	while (1);
            error_count++;
            break;
        }
    }
}


// TODO really good docs on this function
void DAM_init(CAN_HandleTypeDef* gcan, U8 this_module_id, CAN_HandleTypeDef* scan,
			  ADC_HandleTypeDef* adc1, ADC_HandleTypeDef* adc2, ADC_HandleTypeDef* adc3,
			  TIM_HandleTypeDef* tim10, TIM_HandleTypeDef* tim11, TIM_HandleTypeDef* tim14)
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


// TODO add some easy way to set a parameter to the LOCKED_SEND state


// DAM_reset
//  Reset all of the sensor buffers and start the DLM-DAM initialization process
//  over again
void DAM_reset(void)
{
	// All code needed for DLM-DAM reset goes here
	stopTimers();

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
			param->can_param->data = INITIAL_DATA; // Set some initial value
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

	// start collecting data through the timers
	startTimers();
	hasInitialized = TRUE;
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
		if (buffer_full(&param->buffer))
		{
			if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS)
			{
				continue;
			}
			data_in = avg;

			if (apply_analog_sensor_conversion(param->analog_sensor, data_in, &converted_data) != BUFFER_SUCCESS)
			{
				handle_DAM_error(CONVERSION_ERROR);
			}
			// No data cast as we assume params are set up correctly
			// TODO support all data types on the bus
			param->analog_param.can_param->data = converted_data;
			param->analog_param.status = DIRTY;
			fill_analog_subparams(param, converted_data);
		}
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
        if (buffer_full(&param->buffer))
        {
            if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS)
            {
                continue;
            }
            data_in = avg;

            if (apply_can_sensor_conversion(param->can_sensor, param->message_idx, data_in, &converted_data) != BUFFER_SUCCESS)
            {
            	handle_DAM_error(CONVERSION_ERROR);
            }
            // No data cast as we assume params are set up correctly
            // TODO support all data types on the bus
            param->can_param.can_param->data = converted_data; // fill the data
            param->can_param.status = DIRTY;
            fill_can_subparams(param, converted_data);
        }
        param++;
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
							handle_DAM_error(TBD_ERROR);
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


/* DAM_main_task
 * Main task state machine
 */
void DAM_main_task (void)
{
    // Must have started buffer service task and tx task
    while (1)
    {
    	if (!hasInitialized) // waiting for initialization
    	{
    		osDelay(1);
    	}
    	else
    	{
    		// TODO add some check for if all of the buckets are done configuring
    		// and give some sort of external ok for devs to look at
    		ADC_sensor_service();
    		sensorCAN_service();
    	}
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
        handle_DAM_error(TBD_ERROR);
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
        handle_DAM_error(TBD_ERROR);
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

   else handle_DAM_error(TBD_ERROR); // This case shouldnt happen
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
            handle_DAM_error(TBD_ERROR);
        }
        osDelay(1);
    }
}


// TODO: add timer interrupt to ensure IDLEtask runs??



