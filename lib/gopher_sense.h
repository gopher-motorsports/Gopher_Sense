// gopher_sense.h
//  TODO DOCS

#ifndef GOPHER_SENSE_H
#define GOPHER_SENSE_H

#include "module_hw_config.h"
#include "adc_lib.h"
#include "gsense_structs.h"
#include "base_types.h"
#include "GopherCAN.h"

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

} GSENSE_ERROR_STATE;

#define GSENSE_SUCCESS 0
#define GSESNE_FAIL -1

// LED #defines
#define LED_ERROR_OFF_TIME 800
#define LED_ERROR_BLINK_TIME 200
#define LED_NO_ERROR_BLINK_TIME 500
#define LED_NO_LOGGER_COMMS_BLINK_TIME 2000

#define GET_U16_MSB(u16) ((u16 & 0xFF00) >> 8)
#define GET_U16_LSB(u16) (u16 & 0xFF)

//---------------Function Prototypes---------------
#if NEED_ADC
#if NEED_HW_TIMER
GSENSE_ERROR_STATE gsense_init (CAN_HandleTypeDef* gcan, ADC_HandleTypeDef* adc1,
						       	ADC_HandleTypeDef* adc2, ADC_HandleTypeDef* adc3,
						       	TIM_HandleTypeDef* tim10, GPIO_TypeDef* stat_led_GPIOx,
							   	U16 stat_led_Pin);
#else // NEED_HW_TIMER
GSENSE_ERROR_STATE gsense_init (CAN_HandleTypeDef* gcan, ADC_HandleTypeDef* adc1,
						       	ADC_HandleTypeDef* adc2, ADC_HandleTypeDef* adc3,
						    	GPIO_TypeDef* stat_led_GPIOx, U16 stat_led_Pin);
#endif // NEED_HW_TIMER
#else // NEED_ADC
GSENSE_ERROR_STATE gsense_init (CAN_HandleTypeDef* gcan, GPIO_TypeDef* stat_led_GPIOx,
						       	U16 stat_led_Pin);
#endif // NEED_ADC

void set_all_param_sending(boolean enabled);
S8 lock_param_sending(CAN_INFO_STRUCT* can_param);
S8 update_and_queue_param_float(FLOAT_CAN_STRUCT* can_param, float f);
S8 update_and_queue_param_u32(U32_CAN_STRUCT* can_param, U32 u32);
S8 update_and_queue_param_u16(U16_CAN_STRUCT* can_param, U16 u16);
S8 update_and_queue_param_u8(U8_CAN_STRUCT* can_param, U8 u8);
void gsense_reset(void);
void gsense_main_task(void* param);
void log_complete(MODULE_ID sender, void* parameter,
                  U8 UNUSED1, U8 UNUSED2, U8 UNUSED3, U8 UNUSED4);

#endif // GOPHER_SENSE_H

// End of gopher_sense.h
