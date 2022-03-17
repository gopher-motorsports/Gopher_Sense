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
SENSOR_H_FILE = 'gopher_sense_TEMPLATE.h.jinja2'
SENSOR_C_FILE = 'gopher_sense_TEMPLATE.c.jinja2'
HWCONFIG_C_FILE = 'hwconfig_TEMPLATE.c.jinja2'
HWCONFIG_H_FILE = 'hwconfig_TEMPLATE.h.jinja2'

def C_ize_Name(name):
    return name.replace(' ', '_').lower()

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


class Param():
    def __init__(self, param_name, filtered_params, buffer_size, producer, filter, sen_name,  sen_output ):
        self.param_name = param_name
        self.filtered_params = filtered_params
        self.buffer_size = buffer_size
        self.producer = producer
        self.filter = filter #(type, value) tuple
        self.sensor_name = sen_name
        self.sensor_output = sen_output
        self.bucket_id = 420
        self.bucket_loc = 69

class Module():
    def __init__(self, name, params, analog_sensors, can_sensors):
        self.analog_sensors = analog_sensors
        self.can_sensors = can_sensors

        self.name = name

        self.adc1_params = []
        self.adc2_params = []
        self.adc3_params = []

        self.can_params = []


        self.params = params
        for p in self.params:
            param = self.params[p]
            producer = param['produced_by']
            
            # Filtered params would go here if they existed

            p = Param(p, None, param['num_samples_buffered'], producer, None, param['sensor']['name'], param['sensor']['output'] )
            if ("ADC1" in producer):
                self.adc1_params.append(p)
            elif ("ADC2" in producer):
                self.adc2_params.append(p)
            elif ("ADC3" in producer):
                self.adc3_params.append(p)
            elif ("CAN" in producer):
                self.can_params.append(p)

        # This sorting is to the ADC channels are in order
        self.adc1_params.sort(key=lambda param:int(param.producer[7:]), reverse=False)
        self.adc2_params.sort(key=lambda param:int(param.producer[7:]), reverse=False)
        self.adc3_params.sort(key=lambda param:int(param.producer[7:]), reverse=False)


    def getSensorName(self, param_sensor_name):
        for asensor in self.analog_sensors:
            if param_sensor_name == asensor.sensorID:
                return asensor.name
                
        for csensor in self.can_sensors:
            if param_sensor_name == csensor.sensorID:
                return csensor.name
        
        return None


    def getDependencyMessageIndex(self, name, dependency):
        sensor = None
        for s in self.can_sensors:
            if s.name == name:
                sensor = s
                break
        if sensor == None:
            return None
        idx = 0
        for message in sensor.messages:
            if sensor.messages[message]['output_measured'] == dependency:
                return idx
            idx += 1
        return None

class Bucket():
    def __init__(self, name, id, frequency, params):
        self.name = name
        self.id = id
        self.ms_between_req = int(1000 / frequency)
        self.params = params

# convienient container for data
class AnalogSensor():
    def __init__(self, sensorID, name, outputs, analog):
        self.sensorID = sensorID
        self.name = C_ize_Name(name)
        self.outputs = outputs
        self.analog = analog
        self.table = self.analog['table']
        self.tableEntries = ([],[]) #entries in order (independent, dependent)
        self.numTableEntries = None
        if self.table != None:
            self.numTableEntries = int(len(self.table['entries'])/2) #always even
            for i in range(self.numTableEntries):
                self.tableEntries[0].append(self.table['entries']["independent{}".format(i+1)])
                self.tableEntries[1].append(self.table['entries']["dependent{}".format(i+1)])

# convienient container for data
class CANSensor():
    def __init__(self, sensorID, name, outputs, byte_order, messages):
        self.sensorID = sensorID
        self.name = C_ize_Name(name)
        self.outputs = outputs
        self.byte_order = byte_order
        self.messages = messages
        self.numMessages = len(messages)

def main():
    argv = sys.argv
    if len(argv) < 2:
        print("Pass the path to the hardware config file: somepath\\some_module_hwconfig.yaml")
        sys.exit()

    print("Gopher Motorsports Sensor Cannon")
    sensorFile = open(SENSOR_CONFIG_FILE)
    configFile = open(argv[1])
    configFileName = argv[1].split('\\')[-1].replace(".yaml", "")
    sensor_raw = yaml.full_load(sensorFile)
    hwconfig_raw = yaml.full_load(configFile)
    hwconfig_munch = munch.Munch(hwconfig_raw)
    sensors_munch = munch.Munch(sensor_raw)
    sensors = sensors_munch.sensors

    # define sensor objects
    analog_sensors = []
    can_sensors = []
    for s in sensors:
        sensor = sensors[s]
        if 'analog' in sensor['sensor_type']:
            a = AnalogSensor(s, sensor['name_english'], sensor['outputs'], sensor['sensor_type']['analog'])
            analog_sensors.append(a)
        if 'CAN' in sensor['sensor_type']:
            c = CANSensor(s, sensor['name_english'], sensor['outputs'] , \
                          sensor['sensor_type']['CAN']['byte_order'], sensor['sensor_type']['CAN']['messages'])
            can_sensors.append(c)
        # more sensors can be added here


    # write the sensor templates
    os.makedirs(OUTPUT_DIRECTORY, exist_ok=True)
    print("Generating ", SENSOR_H_FILE)
    with open(os.path.join(TEMPLATES_DIRECTORY, SENSOR_H_FILE)) as file_:
        template = Template(file_.read())
        output = template.render(analog_sensors=analog_sensors, can_sensors=can_sensors)
        filename = "gopher_sense.h"
        with open(os.path.join(OUTPUT_DIRECTORY, filename), "w") as fh:
            fh.write(output)
    print("Generating ", SENSOR_C_FILE)
    with open(os.path.join(TEMPLATES_DIRECTORY, SENSOR_C_FILE)) as file_:
        template = Template(file_.read())
        output = template.render(analog_sensors=analog_sensors, can_sensors=can_sensors)
        filename = "gopher_sense.c"
        with open(os.path.join(OUTPUT_DIRECTORY, filename), "w") as fh:
            fh.write(output)

    module = Module(hwconfig_munch.module_name, hwconfig_munch['parameters_produced'], analog_sensors, can_sensors)
    buckets = []
    for _b in hwconfig_munch.buckets:
        b = hwconfig_munch.buckets[_b]
        bucket_params = [bp for bp in b['parameters']]
        buckets.append(Bucket(_b, b['id'], b['frequency_hz'], bucket_params))
        
        
    # TODO some error if the link fails
    # link all the params with where they are in the buckets
    for param in module.adc1_params:
        for bucket in buckets:
            for bucket_param in bucket.params:
                if param.param_name == bucket_param:
                    # this is the bucket to link the param to
                    param.bucket_id = buckets.index(bucket) + 1
                    param.bucket_loc = bucket.params.index(bucket_param)
                    
    for param in module.adc2_params:
        for bucket in buckets:
            for bucket_param in bucket.params:
                if param.param_name == bucket_param:
                    # this is the bucket to link the param to
                    param.bucket_id = buckets.index(bucket) + 1
                    param.bucket_loc = bucket.params.index(bucket_param)
                    
    for param in module.adc3_params:
        for bucket in buckets:
            for bucket_param in bucket.params:
                if param.param_name == bucket_param:
                    # this is the bucket to link the param to
                    param.bucket_id = buckets.index(bucket) + 1
                    param.bucket_loc = bucket.params.index(bucket_param)
                    
    for param in module.can_params:
        for bucket in buckets:
            for bucket_param in bucket.params:
                if param.param_name == bucket_param:
                    # this is the bucket to link the param to
                    param.bucket_id = buckets.index(bucket) + 1
                    param.bucket_loc = bucket.params.index(bucket_param)
                    
        

    print("Generating ", HWCONFIG_C_FILE)
    with open(os.path.join(TEMPLATES_DIRECTORY, HWCONFIG_C_FILE)) as file_:
        template = Template(file_.read())
        output = template.render(module=module, buckets=buckets, configFileName=configFileName)
        print("Configuring DAM from:", configFileName)
        filename = "dam_hw_config.c"
        with open(os.path.join(OUTPUT_DIRECTORY, filename), "w") as fh:
            fh.write(output)
    print("Generating ", HWCONFIG_H_FILE)
    with open(os.path.join(TEMPLATES_DIRECTORY, HWCONFIG_H_FILE)) as file_:
        template = Template(file_.read())
        output = template.render(module=module, buckets=buckets, configFileName=configFileName)
        filename = "dam_hw_config.h"
        with open(os.path.join(OUTPUT_DIRECTORY, filename), "w") as fh:
            fh.write(output)
if __name__ == '__main__':
    main()
    print('Generation Complete.')
