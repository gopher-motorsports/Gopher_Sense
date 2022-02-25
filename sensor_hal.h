/*
 * sensor_hal.h
 *
 *  Created on: Mar 18, 2021
 *      Author: ian
 */

#ifndef INC_SENSOR_HAL_H_
#define INC_SENSOR_HAL_H_

#include "GopherCAN_structs.h"
#include "base_types.h"

#ifndef NULL
#define NULL ((void*)0)
#endif

#define STR_LEN 50

typedef union
{
  U8_CAN_STRUCT u8_struct;
  U16_CAN_STRUCT u16_struct;
  U32_CAN_STRUCT u32_struct;
  U64_CAN_STRUCT u64_struct;
  S8_CAN_STRUCT s8_struct;
  S16_CAN_STRUCT s16_struct;
  S32_CAN_STRUCT s32_struct;
  S64_CAN_STRUCT s64_struct;
  FLOAT_CAN_STRUCT float_struct;
} PARAM_STRUCT;

typedef enum
{
    CLEAN = 0,
    DIRTY = 1,
    LOCKED = 2
} DATA_STATUS;


typedef struct
{
    PARAM_STRUCT param;
    DATA_STATUS  status;
} GENERAL_PARAMETER;


// scales data according to a multiplicative quantization
// and additive offset
typedef struct
{
    float   quantization;
    float   offset;
} DATA_SCALAR;


typedef struct
{
    GENERAL_PARAMETER*  list;
    U16                 len;
} PARAM_LIST;


typedef enum {
    BUCKET_INIT = 0,
    BUCKET_DLM_OK = 1,
    BUCKET_GETTING_DATA = 2,
    BUCKET_REQUESTED = 3,
    BUCKET_SENDING = 4
} BUCKET_STATE;

// describes a bucket of parameters
typedef struct
{
    U8           bucket_id;
    U16          frequency;
    BUCKET_STATE state;
    PARAM_LIST   bucket;
} BUCKET;

typedef enum
{
    VOLTAGE = 0,
    CURRENT = 1,
    RESISTIVE = 2
} ANALOG_SUBTYPE;

typedef struct {
    U32* buffer;
    U16  buffer_size;
    U16  fill_level;
} U32_BUFFER;

typedef struct
{
    float*  independent_vars;
    float*  dependent_vars;
    U16     num_entries;
} TABLE;

typedef enum
{
    LSB = 0,
    MSB = 1
} B_O;


// how to turn raw data into useable measurments
typedef struct
{
	ANALOG_SUBTYPE      input_type;
    float 				rin;
    float 				rdown;
    float				r3v;
    float 				r5v;
    float				rfilt;
    float				rdiv;
    TABLE*              table;
} OUTPUT_MODEL;

// what is this data?
typedef struct
{
    char         output_name[STR_LEN];
    char	     output_unit[STR_LEN];
    DATA_SCALAR  scalar;
    U8           data_size_bits;
} OUTPUT;


// describes an analog sensor
// assumed that there is only 1 output
typedef struct
{
    char            sensor_id[STR_LEN];
    OUTPUT_MODEL    model;
    OUTPUT          output;
} ANALOG_SENSOR;



typedef enum {
    HIGH_PASS = 0,
    LOW_PASS = 1,
	MOVING_AVERAGE = 2,
} FILTER_TYPE;


// describes how to filter a parameter
typedef struct
{
    GENERAL_PARAMETER   filtered_param;
    FILTER_TYPE         filter_type;
    U16                 filter_value;
} FILTERED_PARAM;


// link between gophercan parameter and analog sensor data
typedef struct
{
    GENERAL_PARAMETER   analog_param; // raw data
    ANALOG_SENSOR       analog_sensor;
    FILTERED_PARAM*     filtered_subparams;
    U8                  num_filtered_subparams;
    U32_BUFFER          buffer;
} ANALOG_SENSOR_PARAM;


// describes how to interpret a part of a CAN sensor message
typedef struct
{
  U32       message_id;
  OUTPUT    output;
  U8        data_start;
  U8        data_end;
} SENSOR_CAN_MESSAGE;

// describes a CAN sensor
typedef struct
{
    char                sensor_id[50];
    B_O		            byte_order;
    SENSOR_CAN_MESSAGE* messages;
    U8                  num_messages;
} CAN_SENSOR;


// link between gophercan parameter and CAN sensor data
typedef struct
{
    GENERAL_PARAMETER can_param; // raw data
    CAN_SENSOR        can_sensor;
    U8                message_idx; // which message from the can sensor?
    FILTERED_PARAM*   filtered_subparams;
    U8                num_filtered_params;
    U32_BUFFER        buffer;
} CAN_SENSOR_PARAM;



//Functions



#endif /* INC_SENSOR_HAL_H_ */
