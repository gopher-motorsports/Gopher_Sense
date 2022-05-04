#ifndef GOPHER_SENSE_LIB_H
#define GOPHER_SENSE_LIB_H

#include "gopher_sense_lib.h"
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


// Lib Config
void init_sensor_hal(void);
S8 configLibADC(ADC_HandleTypeDef* ad1, ADC_HandleTypeDef* ad2, ADC_HandleTypeDef* ad3);
S8 configLibTIM(TIM_HandleTypeDef* tim, U16 tim_freq, U16 psc);

void configTimer(TIM_HandleTypeDef* timer, U16 psc, U16 timer_int_freq_hz);
void startDataAq(void);
void stopDataAq(void);
void DAQ_TimerCallback(TIM_HandleTypeDef* timer);
void add_data_to_buffer(ANALOG_SENSOR_PARAM* param_array, volatile U16* sample_buffer, U32 num_params);
void service_scan_rx_buffer(void);
void sensor_can_message_handle (CAN_MSG* message);
void add_scan_message_to_bufffer(CAN_HandleTypeDef* hcan, U32 rx_mailbox);

S8 buffer_full (U32_BUFFER* buffer);
S8 add_to_buffer (U32_BUFFER* buffer, U32 toadd);
S8 reset_buffer (U32_BUFFER* buffer);
S8 average_buffer (U32_BUFFER* buffer, U32* avg);
S8 average_buffer_as_float (U32_BUFFER* buffer, float* avg);
S8 apply_can_sensor_conversion(CAN_SENSOR* sensor, U8 msg_idx, float data_in, float* data_out);
S8 apply_analog_sensor_conversion(ANALOG_SENSOR* sensor, float data_in, float* data_out);

S8 convert_voltage_load (ANALOG_SENSOR* sensor, float data_in, float* data_out);
S8 convert_resistive_load (ANALOG_SENSOR* sensor, float data_in, float* data_out);
S8 convert_current_load (ANALOG_SENSOR* sensor, float data_in, float* data_out);
S8 apply_special_conversions (ANALOG_SENSOR* sensor, float data_in, float* data_out);

S8 interpolate_table_linear (TABLE* table, float data_in, float* data_out);
float interpolate(float x0, float y0, float x1, float y1, float x);
float adc_to_volts(U16 adc_reading, U8 resolution_bits);

S8 apply_filter (U32_BUFFER* buffer, FILTERED_PARAM* filter);





#endif //  GOPHER_SENSE_LIB_H
