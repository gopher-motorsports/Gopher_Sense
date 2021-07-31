#ifndef GOPHER_SENSE_LIB_H
#define GOPHER_SENSE_LIB_H

#include "gopher_sense_lib.h"
#include "gopher_sense.h"
#include "GopherCAN.h"
#include "main.h"



//Buffer
#define BUFFER_ERR -1
#define BUFFER_SUCCESS 1

void init_sensor_hal(void);

void TimerCallback(TIM_HandleTypeDef* timer);

void configLibADC(ADC_HandleTypeDef* ad1, ADC_HandleTypeDef* ad2, ADC_HandleTypeDef* ad3);
void configLibTIM(TIM_HandleTypeDef* t1, U16 t1_freq,
                  TIM_HandleTypeDef* t2, U16 t2_freq,
                  TIM_HandleTypeDef* t3, U16 t3_freq, U16 psc);

void startTimers(void);
void stopTimers(void);


S8 add_to_buffer (U16_BUFFER* buffer, U16 toadd);
S8 buffer_full (U16_BUFFER* buffer);
S8 reset_buffer (U16_BUFFER* buffer);
S8 average_buffer (U16_BUFFER* buffer, U16* avg);
S8 apply_can_sensor_conversion(CAN_SENSOR* sensor, U8 msg_idx, float data_in, float* data_out);
S8 apply_analog_sensor_conversion(ANALOG_SENSOR* sensor, float data_in, float* data_out);
S8 apply_filter (U16_BUFFER* buffer, FILTERED_PARAM* filter);



#endif //  GOPHER_SENSE_LIB_H
