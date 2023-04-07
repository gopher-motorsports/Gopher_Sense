// adc_lib.h
//  Header file for adc_lib.c

#ifndef ADC_LIB_H
#define ADC_LIB_H

#include "module_hw_config.h"
#include "gsense_structs.h"
#include "gopher_sense.h"
#include "GopherCAN.h"
#include "main.h"


//Define error return codes here
#define BUFFER_ERR -1
#define BUFFER_SUCCESS 1
#define CONV_ERR -2
#define RESISTOR_ERR -3
#define BUFFER_EMPTY -4
#define TMR_NOT_CONFIGURED -5
#define ADC_NOT_CONFIGURED -6
#define CONV_SUCCESS 2
#define ADC_LIB_CONFIG_SUCCESS 0

// Set if any ADC is needed
#define NEED_ADC ((NUM_ADC1_PARAMS > 0) || (NUM_ADC2_PARAMS > 0) || (NUM_ADC3_PARAMS > 0))

// Set if ADC needed and HW timer has been requested
#define NEED_HW_TIMER NEED_ADC && HW_TIMER_REQUESTED

// Function prototypes
#if NEED_ADC
S8 configLibADC(ADC_HandleTypeDef* ad1, ADC_HandleTypeDef* ad2, ADC_HandleTypeDef* ad3);
#if NEED_HW_TIMER
void DAQ_TimerCallback(TIM_HandleTypeDef* timer);
S8 configLibTIM(TIM_HandleTypeDef* tim, U16 tim_freq, U16 psc);
#else // NEED_HW_TIMER
void DAQ_UpdateADC();
#endif // NEED_HW_TIMER
#endif // NEED_ADC

void startDataAq(void);
void stopDataAq(void);
void add_data_to_buffer(ANALOG_SENSOR_PARAM* param_array, volatile U16* sample_buffer, U32 num_params);
S8 buffer_full(U16_BUFFER* buffer);
S8 add_to_buffer(U16_BUFFER* buffer, U16 toadd);
S8 reset_buffer(U16_BUFFER* buffer);
S8 reset_buffer(U16_BUFFER* buffer);
S8 average_buffer(U16_BUFFER* buffer, U16* avg);
S8 apply_analog_sensor_conversion(ANALOG_SENSOR* sensor, U16 data_in, float* data_out);

#endif // ADC_LIB_H

// End of adc_lib.h
