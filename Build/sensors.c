// Generated by SensorCannon

#include "sensors.h"

// ********** ANALOG_SENSORS **********

// Sensor definition: Linear Position Sensor 50mm

float linear_pos_sensor_50_table_independent_vars[2] = {
    0.0,
    3.3
    
};
float linear_pos_sensor_50_table_dependent_vars[2] = {
    0.0,
    50.0
    
};
TABLE linear_pos_sensor_50_table = 
{
    .num_entries = 2,
    .independent_vars = linear_pos_sensor_50_table_independent_vars,
    .dependent_vars = linear_pos_sensor_50_table_dependent_vars
};
ANALOG_SENSOR linear_pos_sensor_50 = 
{
    .type = VOLTAGE,
	.conversion_table = linear_pos_sensor_50_table
};


// Sensor definition: Resistive Temp Sensor

float temp_sensor_table_independent_vars[20] = {
    45313.0,
    26114.0,
    15462.0,
    9397.0,
    5896.0,
    3792.0,
    2500.0,
    1707.0,
    1175.0,
    834.0,
    596.0,
    436.0,
    323.0,
    243.0,
    187.0,
    144.0,
    113.0,
    89.0,
    71.0,
    57.0
    
};
float temp_sensor_table_dependent_vars[20] = {
    -40.0,
    -30.0,
    -20.0,
    -10.0,
    0.0,
    10.0,
    20.0,
    30.0,
    40.0,
    50.0,
    60.0,
    70.0,
    80.0,
    90.0,
    100.0,
    110.0,
    120.0,
    130.0,
    140.0,
    150.0
    
};
TABLE temp_sensor_table = 
{
    .num_entries = 20,
    .independent_vars = temp_sensor_table_independent_vars,
    .dependent_vars = temp_sensor_table_dependent_vars
};
ANALOG_SENSOR temp_sensor = 
{
    .type = RESISTIVE,
	.conversion_table = temp_sensor_table
};



// END autogenerated file