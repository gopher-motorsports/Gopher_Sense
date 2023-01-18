// gsense_strucs.h
//  header file that stores all of the structs and some defines that are used
//  all throughout the library

#ifndef GSENSE_STRUCTS_H
#define GSENSE_STRUCTS_H

#include "GopherCAN.h"
#include "base_types.h"

#ifndef NULL
#define NULL ((void*)0)
#endif

typedef enum
{
    NO_SEND_NEEDED = 0,
    SEND_NEEDED = 1,
    LOCKED_SEND = 2
} DATA_STATUS;


typedef struct
{
	CAN_INFO_STRUCT* can_param;
    DATA_STATUS      status;
    U32              ms_between_requests;
} GENERAL_PARAMETER;


typedef struct
{
    GENERAL_PARAMETER*  list;
    U16                 len;
} PARAM_LIST;


typedef enum
{
    VOLTAGE = 0,
    RESISTIVE = 1
} ANALOG_SUBTYPE;


typedef struct {
    U16* buffer;
    U16  buffer_head;
    U16  buffer_size;
    U16  fill_level;
} U16_BUFFER;


typedef struct
{
    float*  independent_vars;
    float*  dependent_vars;
    U16     num_entries;
} TABLE;


typedef struct
{
	ANALOG_SUBTYPE type;
	TABLE*         conversion_table;
} ANALOG_SENSOR;


// link between gophercan parameter and analog sensor data. Averaged data
// from the DMA buffer is put here during the timer interrupt, then the
// main task averages what is in this buffer to put in the GCAN variable
typedef struct
{
    GENERAL_PARAMETER*  gsense_param;
    ANALOG_SENSOR*      analog_sensor;
    U16_BUFFER          buffer;
} ANALOG_SENSOR_PARAM;


#endif // GSENSE_STRUCTS_H

// End of gsense_struct.h
