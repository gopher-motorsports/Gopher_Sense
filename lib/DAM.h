/*
 * DAM.h
 *
 *  Created on: Jun 9, 2021
 *      Author: ian
 */

#ifndef INC_DAM_H_
#define INC_DAM_H_

#include "base_types.h"
#include "sensor_hal.h"
#include "GopherCAN.h"
#include "dam_hw_config.h"
#include "gopher_sense_lib.h"

typedef enum
{
    NO_ERRORS = 0,
    INITIALIZATION_ERROR = 1,
	CONVERSION_ERROR = 2,
	GCAN_TX_FAILED = 3,
	DATA_ASSIGNMENT_ERROR = 4,
	BUCKET_NOT_RECOGNIZED = 5,
	CAN_HANDLE_NOT_RECOGNIZED = 6,
	RX_BUFFER_HANDLE_ERROR = 7,
	TIMER_TO_ADC_ERROR = 8,
	SCAN_DLC_ERROR = 9,
	TASK_EXIT_ERROR = 10,
	SCAN_YAML_CONFIG_ERR = 11,
	SCAN_RX_ERR = 12

} DAM_ERROR_STATE;

#define NEED_HW_TIMER ((NUM_ADC1_PARAMS > 0) || (NUM_ADC2_PARAMS > 0) || (NUM_ADC3_PARAMS > 0))
#define MAX_TIME_BETWEEN_TX_ms 2500

//---------------Function Prototypes---------------
void handle_DAM_error(DAM_ERROR_STATE error_state);
void handle_DAM_LED(void);
DAM_ERROR_STATE DAM_init(CAN_HandleTypeDef* gcan, CAN_HandleTypeDef* scan,
						 ADC_HandleTypeDef* adc1, ADC_HandleTypeDef* adc2, ADC_HandleTypeDef* adc3,
						 TIM_HandleTypeDef* tim10, GPIO_TypeDef* stat_led_GPIOx, U16 stat_led_Pin);
S8 lock_param_sending(CAN_INFO_STRUCT* can_param);
S8 update_and_queue_param_float(FLOAT_CAN_STRUCT* can_param, float f);
S8 update_and_queue_param_u8(U8_CAN_STRUCT* can_param, U8 u8);
S8 update_and_queue_param_u32(U32_CAN_STRUCT* can_param, U32 u32);
void DAM_reset(void);
void complete_DLM_handshake (void);
BUCKET* get_bucket_by_id (U8 bucket_id);
void ADC_sensor_service (void);
void service_ADC(ANALOG_SENSOR_PARAM* adc_params, U32 num_params);
void sensorCAN_service (void);
void fill_can_subparams (CAN_SENSOR_PARAM* param, float newdata);
void fill_analog_subparams (ANALOG_SENSOR_PARAM* param, float newdata);
S8 fill_gcan_param_data(CAN_INFO_STRUCT* can_param, float data);


void send_bucket_task (void* pvParameters);
void DAM_main_task(void* param);
void gopherCAN_tx_service_task (void);
void gopherCAN_rx_buffer_service_task (void);

void send_bucket_params (U8 sender, void* param, U8 U1, U8 U2, U8 U3, U8 U4);
void bucket_ok(MODULE_ID sender, void* parameter,
               U8 bucket_id, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
void log_complete(MODULE_ID sender, void* parameter,
        U8 UNUSED1, U8 UNUSED2, U8 UNUSED3, U8 UNUSED4);


#define GET_U16_MSB(u16) ((u16 & 0xFF00) >> 8)
#define GET_U16_LSB(u16) (u16 & 0xFF)

#endif /* INC_DAM_H_ */
