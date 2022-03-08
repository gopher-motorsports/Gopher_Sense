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

// TODO Move this to a config file. Maybe auto generate?
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


typedef enum
{
    WAITING = 0,
    CONFIG = 1,
    NORMAL = 2
} DAM_STATE;

static U64 error_count;
static DAM_STATE dam_state = WAITING;
static DAM_ERROR_STATE latched_error_state = NO_ERRORS;
static boolean hasInitialized = FALSE;


void change_led_state(U8 sender, void* parameter, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
    // this function will set the LED to high or low, depending on remote_param
    // the LED to change is dependent on the parameter stored on this module (*((U16*)parameter))
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
}


// TODO: Error state behavior
void handle_DAM_error (DAM_ERROR_STATE error_state)
{
    latched_error_state = error_state;
    switch (error_state)
    {
        // log the error?
        case INITIALIZATION_ERROR:
        {
            NVIC_SystemReset();
            break;
        }
        case CRITICAL_ERROR:
        {
            NVIC_SystemReset();
            break;
        }
        case CONVERSION_ERROR:
        {
        	//send can error???
        	break;
        }
        default:
        {
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
		bucket->state = BUCKET_INIT;

		param = bucket->param_list.list;
		while (param - bucket->param_list.list < bucket->param_list.len)
		{
			param->status = CLEAN;
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

	hasInitialized = TRUE;
}


// complete_DLM_handshake
//  This function should get called in its own task and will loop through all of the buckets
//  attempting to get the correct configuration details until each bucket has been sent and
//  acked by the DLM
// TODO change this so each bucket is completely independent for more robust code
void complete_DLM_handshake (void)
{
    boolean all_buckets_ok = FALSE;

    while (!all_buckets_ok)
    {
        boolean check_buckets_ok = TRUE;
        BUCKET* bucket = bucket_list;

        while (bucket - bucket_list < NUM_BUCKETS)
        {
            if (bucket->state == BUCKET_DLM_OK)
            {
                continue; // if bucket ok dont do anything
            }
            check_buckets_ok = FALSE;
            send_can_command(PRIO_HIGH, DLM_ID, SET_BUCKET_SIZE, bucket->bucket_id, (U8)bucket->param_list.len, 0, 0);

            GENERAL_PARAMETER* param = bucket->param_list.list;
            while (param - bucket->param_list.list < bucket->param_list.len)
            {
                send_can_command(PRIO_HIGH, DLM_ID, ADD_PARAM_TO_BUCKET, bucket->bucket_id,
                                 GET_U16_MSB(param->can_param->param_id),
								 GET_U16_LSB(param->can_param->param_id), 0);
                param++;
            }

            bucket++;
            osDelay(INIT_TX_DELAY_TIME_ms); // Delay to avoid flooding the TX_queue
        }
        all_buckets_ok = check_buckets_ok;
    }

    // Assign the buckets to the correct frequencies after all buckets OK
    BUCKET* bucket = bucket_list;
    while (bucket - bucket_list < NUM_BUCKETS)
    {
        send_can_command(PRIO_HIGH, DLM_ID, ASSIGN_BUCKET_TO_FRQ,
                         bucket->bucket_id,
						 GET_U16_MSB(bucket->frequency),
						 GET_U16_LSB(bucket->frequency), 0);
        bucket->state = BUCKET_GETTING_DATA;
        bucket++;
    }

    // TODO add an ack in the interaction to make sure the bucket frequency is sent

    // DLM handshake is complete. Start acquiring data
    startTimers();
    dam_state = NORMAL;
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
//  one task for each bucket. This will loop through and send each bucket parameter when the bucket is
//  requested and bucket->state is changed
void send_bucket_task (void* pvParameters)
{
    BUCKET* bucket = (BUCKET*) pvParameters;

    while(1)
    {
    	while (bucket->state != BUCKET_REQUESTED) taskYIELD();

		// send the bucket parameters
    	GENERAL_PARAMETER* param = bucket->param_list.list;
    	while (param - bucket->param_list.list < bucket->param_list.len)
		{
			if (param->status < DIRTY)
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
				param->status = CLEAN;
		   }
			param++;
		}

		bucket->state = BUCKET_GETTING_DATA;
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
        switch (dam_state)
        {
            case WAITING:
            {
            	osDelay(1);
            	break;
            }
            case CONFIG:
            {
                DAM_reset(); // reset the DAM process if the DLM sends this request
                complete_DLM_handshake();
                break;
            }
            case NORMAL:
            {
                ADC_sensor_service();
                sensorCAN_service();
                break;
            }
            default:
            {
                handle_DAM_error(TBD_ERROR);
            }
        }

    }
}


//*************** GopherCAN callbacks *****************
/* send_bucket_params
 * Handler for the SEND_BUCKET_PARAMS gopherCAN command
 * sets the DAM into configuration state
 */
void send_bucket_params (U8 sender, void* param, U8 U1, U8 U2, U8 U3, U8 U4)
{
    if (sender != DLM_ID) return;

    dam_state = CONFIG;
}


/* bucket_ok
 * Handler for the BUCKET_OK gopherCAN command
 * sets the passed bucket state to OK
 */
void bucket_ok(MODULE_ID sender, void* parameter,
               U8 bucket_id, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
    if (sender != DLM_ID) return;

    BUCKET* bucket = get_bucket_by_id(bucket_id);
    if (bucket != NULL && bucket->state <= BUCKET_DLM_OK )
    {
    	// dont reset the state of bucket getting data/sending
        bucket->state = BUCKET_DLM_OK;
    }
    else
    {
        handle_DAM_error(TBD_ERROR);
    }
}


/* bucket_requested
 * Handler for the RequestBucket gopherCAN command
 * Creates rtos task to handle sending the bucket
 */
void bucket_requested (MODULE_ID sender, void* parameter,
                       U8 bucket_id, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
    BUCKET* bucket = get_bucket_by_id(bucket_id);
    if (bucket == NULL || bucket->state < BUCKET_GETTING_DATA)
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



