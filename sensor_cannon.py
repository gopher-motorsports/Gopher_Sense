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
    def __init__(self, param_name, filtered_params, buffer_size, producer, filter, dependency ):
        self.param_name = param_name
        self.filtered_params = filtered_params
        self.buffer_size = buffer_size
        self.producer = producer
        self.filter = filter #(type, value) tuple
        self.dependency = dependency
        self.bucket_id = 420
        self.bucket_loc = 69

class Module():
    def __init__(self, name, adc_input, can_input, params, analog_sensors, can_sensors):
        self.analog_sensors = analog_sensors
        self.can_sensors = can_sensors

        self.name = name
        self. adc_channels = adc_input
        self.can_input = can_input

        self.adc1_params = []
        self.adc2_params = []
        self.adc3_params = []

        self.can_params = []


        self.params = params
        for p in self.params:
            param = self.params[p]
            producer = param['produced_by']

            filtered_params = []
            if param['filter_subparams']:
                for f_p in param['filter_subparams']:
                    fp = param['filter_subparams'][f_p]
                    filteredP = Param(f_p, [], 0, producer, (fp['filter_type'].upper(),fp['filter_value']), None)
                    filtered_params.append(filteredP)

            p = Param(p, filtered_params, param['buffering']['num_samples_buffered'], producer, None, param['sensor_output'] )
            if ("ADC1" in producer):
                self.adc1_params.append(p)
            elif ("ADC2" in producer):
                self.adc2_params.append(p)
            elif ("ADC3" in producer):
                self.adc3_params.append(p)
            elif ("CAN" in producer):
                self.can_params.append(p)

        # TODO make sure this sorting works, it is very important when running DMA so the sensor signals
        # do not get swapped around
        self.adc1_params.sort(key=lambda param:param.producer, reverse=True)
        self.adc2_params.sort(key=lambda param:param.producer, reverse=True)
        self.adc3_params.sort(key=lambda param:param.producer, reverse=True)


    def getSensorName(self, id):
        aid = None
        cid = None
        if id in self.adc_channels:
            aid = self.adc_channels[id]['sensor']
        if id in self.can_input:
            cid = self.can_input[id]['sensor']
        for asensor in self.analog_sensors:
            if asensor.sensorID == aid:
                return asensor.name
        for csensor in self.can_sensors:
            if csensor.sensorID == cid:
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
        self.runCoherenceCheck()
#     def getOutputQuantization(self, output):
#         if output in self.outputs:
#             return self.outputs[output]['quantization']
#         else:
#             return None
#     def getOutputOffset(self, output):
#         if output in self.outputs:
#             return self.outputs[output]['offset']
#         else:
#             return None
    def runCoherenceCheck(self):
        # TODO check for messages that output an output not listed or if outputs arent produced by messages
        pass

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

    module = Module(hwconfig_munch.module_name, hwconfig_munch['data_input_methods']['adc_channels'], \
                    hwconfig_munch['data_input_methods']['can_sensors'], hwconfig_munch['parameters_produced'], analog_sensors, can_sensors)
    buckets = []
    for _b in hwconfig_munch.buckets:
        b = hwconfig_munch.buckets[_b]
        bucket_params = [bp for bp in b['parameters']]
        buckets.append(Bucket(_b, b['id'], b['frequency_hz'], bucket_params))
        
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
