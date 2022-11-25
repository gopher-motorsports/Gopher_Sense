// TODO DOCS

#ifndef INC_SENSOR_HAL_H_
#define INC_SENSOR_HAL_H_

#include "GopherCAN_structs.h"
#include "base_types.h"

#ifndef NULL
#define NULL ((void*)0)
#endif

#define STR_LEN 50

typedef enum
{
    CLEAN = 0,
    DIRTY = 1,
    LOCKED_SEND = 2
} DATA_STATUS;


typedef struct
{
	CAN_INFO_STRUCT* can_param;
    DATA_STATUS      status;
    U32              last_tx;
} GENERAL_PARAMETER;


typedef struct
{
    GENERAL_PARAMETER*  list;
    U16                 len;
} PARAM_LIST;


typedef enum {
    BUCKET_CONFIG_INIT = 0,
    BUCKET_CONFIG_SENDING_PARAMS = 1,
	BUCKET_GETTING_DATA = 2,
	BUCKET_SENDING = 3
} BUCKET_STATE;


// describes a bucket of parameters
typedef struct
{
    U8           bucket_id; // TODO this can be deleted when buckets are less important
    U16          ms_between_req;
    U32          last_send;
    BUCKET_STATE state;
    PARAM_LIST   param_list;
} BUCKET;


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
	TABLE          conversion_table;
} ANALOG_SENSOR;


// link between gophercan parameter and analog sensor data
typedef struct
{
    GENERAL_PARAMETER*  bucket_param;
    ANALOG_SENSOR*      analog_sensor;
    U16_BUFFER          buffer;
} ANALOG_SENSOR_PARAM;



#endif /* INC_SENSOR_HAL_H_ */
