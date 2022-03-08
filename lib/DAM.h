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
    CAN_ERROR = 2,
    RUNTIME_ERROR = 3,
    CRITICAL_ERROR = 4,
	CONVERSION_ERROR = 5,
    TBD_ERROR = 0xAA,

} DAM_ERROR_STATE;



//---------------Function Prototypes---------------
void handle_DAM_error(DAM_ERROR_STATE error_state);
void DAM_init(CAN_HandleTypeDef* gcan, U8 this_module_id, CAN_HandleTypeDef* scan,
			  ADC_HandleTypeDef* adc1, ADC_HandleTypeDef* adc2, ADC_HandleTypeDef* adc3,
			  TIM_HandleTypeDef* tim10, TIM_HandleTypeDef* tim11, TIM_HandleTypeDef* tim14);
void DAM_reset(void);
void complete_DLM_handshake (void);
BUCKET* get_bucket_by_id (U8 bucket_id);
void ADC_sensor_service (void);
void service_ADC(ANALOG_SENSOR_PARAM* adc_params, U32 num_params);
void sensorCAN_service (void);
void fill_can_subparams (CAN_SENSOR_PARAM* param, float newdata);
void fill_analog_subparams (ANALOG_SENSOR_PARAM* param, float newdata);


void send_bucket_task (void* pvParameters);
void DAM_main_task(void);
void gopherCAN_tx_service_task (void);
void gopherCAN_rx_buffer_service_task (void);

void send_bucket_params (U8 sender, void* param, U8 U1, U8 U2, U8 U3, U8 U4);
void bucket_ok(MODULE_ID sender, void* parameter,
               U8 bucket_id, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
void bucket_requested (MODULE_ID sender, void* parameter,
                       U8 bucket_id, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);


#define GET_U16_MSB(u16) ((u16 & 0xFF00) >> 8)
#define GET_U16_LSB(u16) (u16 & 0xFF)

#endif /* INC_DAM_H_ */
