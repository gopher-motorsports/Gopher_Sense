'''
Created on Apr 3, 2021

@author: ian
'''

import sys
import os
import yaml
import munch
from jinja2 import Template

TEMPLATES_DIRECTORY = "Templates"
OUTPUT_DIRECTORY = "Build"
SENSOR_CONFIG_FILE = "sensors.yaml"
SENSOR_H_FILE = 'sensors_TEMPLATE.h.jinja2'
SENSOR_C_FILE = 'sensors_TEMPLATE.c.jinja2'
HWCONFIG_C_FILE = 'hwconfig_TEMPLATE.c.jinja2'
HWCONFIG_H_FILE = 'hwconfig_TEMPLATE.h.jinja2'

def NoneOnValueError(d, k):
    try:
        return d[k]
    except(KeyError):
        return None

def getSensorNameFromID(id, sensors):
    for sensor in sensors:
        if id == sensor.sensorID:
            return sensor.name
    return None

# Object for sensors from the sensors.yaml file
class AnalogSensor():
    def __init__(self, name, name_english, analog_subtype, points):
        self.name = name
        self.name_english = name_english
        self.analog_subtype = analog_subtype
        self.points = points
        self.tableEntries = ([],[]) #entries in order (independent, dependent)
        self.numTableEntries = len(points)
        if self.points != None:
            for point in points:
                # remove the parentheses and space, then break the string up into two
                point = point.translate(str.maketrans('', '', '( )'))
                split_point = point.split(',')
                
                # convert both into floating point numbers
                self.tableEntries[0].append(float(split_point[0]))
                self.tableEntries[1].append(float(split_point[1]))

# Objects for the module configuration file
class Module():
    def __init__(self, name, buckets, analog_sensors):
        self.analog_sensors = analog_sensors

        self.name = name

        self.all_params = []
        self.adc1_params = []
        self.adc2_params = []
        self.adc3_params = []
        self.buckets = []
        
        for bucket in buckets:
            # for each bucket, define the name and frequency. Dont pass in the parameters
            # yet as we will link them up later
            b = Bucket(bucket, buckets[bucket]['frequency_hz'])
            self.buckets.append(b)
            
            # convert the dictionary to a list in order to index easier
            param_names = list(buckets[bucket]['parameters'])
            param_vals = list(buckets[bucket]['parameters'].values())
            
            # add the parameters from this bucket to the master list of buckets
            for param_name in param_names:
                param_val = param_vals[param_names.index(param_name)]
                p = Param(param_name, param_val['ADC'], param_val['sensor'], param_val['samples_buffered'])
                
                # add the details about what bucket this is in to the parameter
                p.bucket_loc = param_names.index(param_name)
                p.bucket_name = b.name
                
                # note which ADC is generating this parameter. If the ADC is 'NON_ADC' then items
                # just wont get added to any of the lists and must be generated manually
                if ("ADC1" in p.ADC):
                    self.adc1_params.append(p)
                elif ("ADC2" in p.ADC):
                    self.adc2_params.append(p)
                elif ("ADC3" in p.ADC):
                    self.adc3_params.append(p)
                    
                # also keep a running list of all of the parameters in this bucket
                b.params.append(p)

        # This sorting is to the ADC channels are in order
        self.adc1_params.sort(key=lambda param:int(param.ADC[7:]), reverse=False)
        self.adc2_params.sort(key=lambda param:int(param.ADC[7:]), reverse=False)
        self.adc3_params.sort(key=lambda param:int(param.ADC[7:]), reverse=False)

class Bucket():
    def __init__(self, name, frequency):
        self.name = name
        self.ms_between_req = int(1000 / frequency)
        self.params = []
        
class Param():
    def __init__(self, param_name, ADC, sensor_name, samples_buffered):
        self.param_name = param_name
        self.ADC = ADC
        self.sensor_name = sensor_name
        self.samples_buffered = samples_buffered
        self.bucket_name = "[GCAN Param Not Found in Bucket]"
        self.bucket_loc = 1337

def main():
    argv = sys.argv
    if len(argv) < 2:
        print("Pass the path to the hardware config file: somepath\\some_module_hwconfig.yaml")
        sys.exit()

    # Get the details from all the required .yamls
    print("Gopher Motorsports Sensor Cannon")
    sensorFile = open(SENSOR_CONFIG_FILE)
    configFile = open(argv[1])
    configFileName = argv[1].split('\\')[-1].replace(".yaml", "")
    sensor_raw = yaml.full_load(sensorFile)
    hwconfig_raw = yaml.full_load(configFile)
    hwconfig_munch = munch.Munch(hwconfig_raw)
    sensors_munch = munch.Munch(sensor_raw)
    sensors = sensors_munch.sensors
    
    # run the gcan auto gen script to make sure GopherCAN_ids.c/h is up to date
    # TODO

    # define sensor objects
    analog_sensors = []
    for s in sensors:
        sensor = sensors[s]
        a = AnalogSensor(s, sensor['name_english'], sensor['analog_subtype'], sensor['points'])
        analog_sensors.append(a)

    # write the sensor templates
    os.makedirs(OUTPUT_DIRECTORY, exist_ok=True)
    print("Generating......", SENSOR_H_FILE)
    with open(os.path.join(TEMPLATES_DIRECTORY, SENSOR_H_FILE)) as file_:
        template = Template(file_.read())
        output = template.render(analog_sensors=analog_sensors)
        filename = "sensors.h"
        with open(os.path.join(OUTPUT_DIRECTORY, filename), "w") as fh:
            fh.write(output)

    print("Generating......", SENSOR_C_FILE)
    with open(os.path.join(TEMPLATES_DIRECTORY, SENSOR_C_FILE)) as file_:
        template = Template(file_.read())
        output = template.render(analog_sensors=analog_sensors)
        filename = "sensors.c"
        with open(os.path.join(OUTPUT_DIRECTORY, filename), "w") as fh:
            fh.write(output)

    # define the module and start parsing the buckets
    module = Module(hwconfig_munch.module_name, hwconfig_munch['buckets'], analog_sensors)
    
    # write the HW config files
    print("Generating......", HWCONFIG_C_FILE)
    with open(os.path.join(TEMPLATES_DIRECTORY, HWCONFIG_C_FILE)) as file_:
        template = Template(file_.read())
        output = template.render(module=module, buckets=module.buckets)
        filename = "module_hw_config.c"
        with open(os.path.join(OUTPUT_DIRECTORY, filename), "w") as fh:
            fh.write(output)

    print("Generating......", HWCONFIG_H_FILE)
    with open(os.path.join(TEMPLATES_DIRECTORY, HWCONFIG_H_FILE)) as file_:
        template = Template(file_.read())
        output = template.render(module=module, buckets=module.buckets)
        filename = "module_hw_config.h"
        with open(os.path.join(OUTPUT_DIRECTORY, filename), "w") as fh:
            fh.write(output)
            

if __name__ == '__main__':
    main()
    print('Generation Complete.')
