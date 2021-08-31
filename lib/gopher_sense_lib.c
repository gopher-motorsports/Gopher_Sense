#include "gopher_sense_lib.h"
#include "gopher_sense.h"
#include "GopherCAN.h"
#include "base_types.h"
#include "main.h"

// INCLUDE YOUR HW CONFIG FILE HERE
#include "dam_hw_config.h"


ADC_HandleTypeDef* adc1;
ADC_HandleTypeDef* adc2;
ADC_HandleTypeDef* adc3;

TIM_HandleTypeDef* adc1_timer;
TIM_HandleTypeDef* adc2_timer;
TIM_HandleTypeDef* adc3_timer;

//DMA_HandleTypeDef hdma_adc1;
//DMA_HandleTypeDef hdma_adc2;
//DMA_HandleTypeDef hdma_adc3;

static volatile U16 adc1_sample_buffer[NUM_ADC1_PARAMS];
static volatile U16 adc2_sample_buffer[NUM_ADC2_PARAMS];
static volatile U16 adc3_sample_buffer[NUM_ADC3_PARAMS];


#define TIM_CLOCK_BASE_FREQ 16000000
#define TIM_MAX_VAL 65536


//************ Initialize the library ****************
void init_sensor_hal (void) {
    init_analog_sensors();
    init_can_sensors();
    init_adc1_params();
    init_adc2_params();
    init_adc3_params();
    init_can_params();
    init_buckets();
}

//******************* ADC Config *******************
void configLibADC(ADC_HandleTypeDef* ad1, ADC_HandleTypeDef* ad2, ADC_HandleTypeDef* ad3)
{
    adc1 = ad1;
    adc2 = ad2;
    adc3 = ad3;
}

void  HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc_handle){
    // stop the DMA and start the timer
    HAL_ADC_Stop_DMA(adc_handle);
    U16_BUFFER* buf;

    if (adc_handle == adc1) {
        HAL_TIM_Base_Start_IT(adc1_timer);
        for (U8 i = 0; i < NUM_ADC1_PARAMS; i++) {
            buf = &adc1_sensor_params[i].buffer;
            add_to_buffer(buf, adc1_sample_buffer[i]);
        }
    }
    else if (adc_handle == adc2) {
        HAL_TIM_Base_Start_IT(adc2_timer);
        for (U8 i = 0; i < NUM_ADC2_PARAMS; i++) {
            buf = &adc2_sensor_params[i].buffer;
            add_to_buffer(buf, adc2_sample_buffer[i]);
        }
    }
    else if (adc_handle == adc3) {
        HAL_TIM_Base_Start_IT(adc3_timer);
        for (U8 i = 0; i < NUM_ADC3_PARAMS; i++) {
            buf = &adc3_sensor_params[i].buffer;
            add_to_buffer(buf, adc3_sample_buffer[i]);
        }
    }
}


//******************* Timer interaction *******************
void configLibTIM(TIM_HandleTypeDef* t1, U16 t1_freq,
                  TIM_HandleTypeDef* t2, U16 t2_freq,
                  TIM_HandleTypeDef* t3, U16 t3_freq, U16 psc)
{
    adc1_timer = t1;
    adc2_timer = t2;
    adc3_timer = t3;
    configTimer(adc1_timer, psc, t1_freq);
    configTimer(adc2_timer, psc, t2_freq);
    configTimer(adc3_timer, psc, t3_freq);
}


void configTimer(TIM_HandleTypeDef* timer, U16 psc,  U16 timer_int_freq_hz) {
    __HAL_TIM_DISABLE(timer);
    __HAL_TIM_SET_COUNTER(timer, 0);
    // Maybe look at this?
    U32 reload;
    do {
        reload = (TIM_CLOCK_BASE_FREQ/psc) / timer_int_freq_hz;
        psc *= 2;
    } while (reload > TIM_MAX_VAL);

    __HAL_TIM_SET_PRESCALER(timer, psc);
    __HAL_TIM_SET_AUTORELOAD(timer, reload);
    __HAL_TIM_ENABLE(timer);
}


void startTimers (void) {
    HAL_TIM_Base_Start_IT(adc1_timer);
    HAL_TIM_Base_Start_IT(adc2_timer);
    HAL_TIM_Base_Start_IT(adc3_timer);
}


void stopTimers (void) {
    HAL_TIM_Base_Stop_IT(adc1_timer);
    HAL_TIM_Base_Stop_IT(adc2_timer);
    HAL_TIM_Base_Stop_IT(adc3_timer);
    __HAL_TIM_SET_COUNTER(adc1_timer, 0);
    __HAL_TIM_SET_COUNTER(adc2_timer, 0);
    __HAL_TIM_SET_COUNTER(adc3_timer, 0);
}


// Call this inside the period elapsed callback
void DAQ_TimerCallback (TIM_HandleTypeDef* timer) {

    HAL_TIM_Base_Stop_IT(timer);
    __HAL_TIM_SET_COUNTER(timer, 0);

    if (timer == adc1_timer) {
        HAL_ADC_Start_DMA(adc1, (uint32_t*)adc1_sample_buffer, NUM_ADC1_PARAMS);
    }
    else if (timer == adc2_timer) {
        HAL_ADC_Start_DMA(adc2, (uint32_t*)adc2_sample_buffer, NUM_ADC2_PARAMS);
    }
    else if (timer == adc3_timer) {
        HAL_ADC_Start_DMA(adc3, (uint32_t*)adc3_sample_buffer, NUM_ADC3_PARAMS);
    }
}




//******************* CAN Handling *******************

// Redesign option: pull this into a queue and handle not in an ISR
void sensor_can_message_handle (CAN_HandleTypeDef* hcan, U32 rx_mailbox)
{
    CAN_RxHeaderTypeDef rx_header;
    CAN_MSG message;

    // Get the message
    if (HAL_CAN_GetRxMessage(hcan, rx_mailbox, &rx_header, message.data) != HAL_OK) {
        // Handle errors ?
        return;
    }
    message.rtr_bit = rx_header.RTR;
    message.id = rx_header.ExtId;
    message.dlc = rx_header.DLC;

    // Check the CAN params for a match
    for (U8 i = 0; i < NUM_CAN_SENSOR_PARAMS; i++) {

        CAN_SENSOR_PARAM* param = &can_sensor_params[i];
        CAN_SENSOR sensor = param->can_sensor;
        SENSOR_CAN_MESSAGE can_info = sensor.messages[param->message_idx];

        // check for ID match between this param message id and the message id
        if (can_info.message_id == message.id) {
            U16 data = 0;
            U8 shift = 0;
            // Get correct data based on byte order
            if (sensor.byte_order == LSB) {

                for (U8 b = can_info.data_start; b <= can_info.data_end; b++) {
                    data &= message.data[b] << shift;
                    shift += 8;
                }
            }

            else if (sensor.byte_order == MSB) {
                for (U8 b = can_info.data_end; b >= can_info.data_start; b--) {
                    data &= message.data[b] << shift;
                    shift += 8;
                }
            }
            // Add the data to the buffer
            add_to_buffer(&param->buffer, data);

        }
    }
}




//******************* Buffer interaction *******************

// Note: Semaphore probably not needed for buffer interaction because reset is atomic

S8 buffer_full (U16_BUFFER* buffer) {
    if (buffer == NULL) {
        return BUFFER_ERR;
    }
    return buffer->fill_level == buffer->buffer_size;
}

S8 add_to_buffer (U16_BUFFER* buffer, U16 toadd) {
    if (buffer == NULL) {
        return BUFFER_ERR;
    }

    if (buffer_full(buffer)) {
        return BUFFER_ERR;
    }

    buffer->buffer[buffer->fill_level] = toadd;
    buffer->fill_level++;
    return BUFFER_SUCCESS;
}

S8 reset_buffer (U16_BUFFER* buffer) {
    if (buffer == NULL) {
        return BUFFER_ERR;
    }

    buffer->fill_level = 0;
    return BUFFER_SUCCESS;
}

// Could average up to the fill level, returns error for now
S8 average_buffer (U16_BUFFER* buffer, U16* avg) {
    if (buffer == NULL || !buffer_full(buffer)) {
        return BUFFER_ERR;
    }
    U16 calc_avg = 0;
    for (U16 i = 0; i < buffer->buffer_size; i++) {
        calc_avg += buffer->buffer[i];
    }

    *avg = calc_avg / buffer->buffer_size;
    return BUFFER_SUCCESS;

}


S8 apply_can_sensor_conversion (CAN_SENSOR* sensor, U8 msg_idx, float data_in, float* data_out) {
    // TODO: Data conversion specifics
    *data_out = data_in;
    return BUFFER_SUCCESS;
}

S8 apply_analog_sensor_conversion (ANALOG_SENSOR* sensor, float data_in, float* data_out) {
    // TODO: Data conversion specifics
    *data_out = data_in;
    return BUFFER_SUCCESS;
}

S8 apply_filter (U16_BUFFER* buffer, FILTERED_PARAM* filter) {
    // TODO - figure out how to do software filtering
    return BUFFER_SUCCESS;
}





