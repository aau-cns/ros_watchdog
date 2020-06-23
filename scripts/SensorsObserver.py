#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import configparser
import os
from enum import Enum

class SensorStatus(Enum):    # RosWatchdog.status
    OK = 0                    # -> OK
    ERROR = 1                 # -> ABORT


class Sensor(object):
    def __init__(self, name, restart_script='', restart_attempts=1):
        pass
        self.name = name
        self.restart_script = restart_script
        self.restart_attempts = restart_attempts
        self.clear_stats()


    def restart(self):
        self.num_restarts += 1

        print('restart sensor ' + str(self.name) + ' num attempt:[' + str(self.num_restarts) + ']')
        print(' -- running: ' + str(self.restart_script))


    def get_status(self):
        if self.num_restarts > self.restart_attempts:
            self.status = SensorStatus.ERROR

        return self.status

    def clear_stats(self):
        self.num_restarts = 0
        self.status = SensorStatus.OK


class SensorsObserver(object):
    def __init__(self, sensors_cfg_file, verbose = True):
        assert (os.path.exists(sensors_cfg_file))
        self.bVerbose = verbose

        self.observers = {}
        config = configparser.ConfigParser()
        config.sections()
        config.read(sensors_cfg_file)
        self.items = config.items()

        # the first element is default section!
        if len(self.items) < 2:
            print("ERROR: no sensors objects in " + str(sensors_cfg_file))

        for key, section in self.items:
            if key != 'DEFAULT':
                if self.bVerbose:
                    print(key)
                # read configuration:
                self.observers[key] = Sensor(name=key,
                                           restart_script=str(section.get('restart_script', '')),
                                           restart_attempts=str(section.get('restart_attempts', 1))
                                           )



    def exists(self, node_name):
        return self.observers.has_key(node_name)

    def start_observation(self):
        pass

    def stop_observation(self):
        pass

    def get_status(self):
        status = {}
        for key, val in self.observers.items():
            status[key] = val.get_status()

        return status
