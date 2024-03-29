#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import configparser
import os
from enum import Enum
from subprocess import call

from observer.Observer import Observer, ObserverStatus, ObserverSeverity

class SensorStatus(Enum):    # RosWatchdog.status
    OK = 0                    # -> OK
    ERROR = 1                 # -> ABORT


class Sensor(Observer):

    def __init__(
            self,
            name,
            observer_id,
            dirname='',
            restart_script='',
            restart_attempts=1,
            verbose=False
            ):

        # initialize super
        super(Sensor, self).__init__(name, observer_id, 0.0, verbose)

        self.dirname = dirname
        self.restart_script = restart_script
        self.restart_attempts = restart_attempts
        self.clear_stats()
        pass


    def restart(self):
        self.num_restarts += 1

        print('drivers sensor ' + str(self.name) + ' num attempt:[' + str(self.num_restarts) + ']')
        file = os.path.join(self.dirname, self.restart_script)
        print(' -- running: ' + str(file))
        rc = call(file, shell=True)
        if rc == 0:
            print('success..')
            return True
        else:
            print('failed...')
            return False

    def get_status(self):
        if self.num_restarts > self.restart_attempts:
            self.status = ObserverStatus.ERROR

        return self.status

    def clear_stats(self):
        self.num_restarts = 0
        self.status = ObserverStatus.NOMINAL


class SensorsObserver(object):
    def __init__(self, sensors_cfg_file, verbose = True):
        assert (os.path.exists(sensors_cfg_file))
        self.bVerbose = verbose

        # setup ID counter
        self.__cnt_id = 1   # always start with 1, 0 --> global

        self.dirname = os.path.dirname(os.path.abspath(sensors_cfg_file))

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
                self.observers[key] = Sensor(
                    name=key,
                    observer_id=self.__cnt_id,
                    dirname=self.dirname,
                    restart_script=str(section.get('restart_script', '')),
                    restart_attempts=int(section.get('restart_attempts', '0')),
                    verbose=verbose
                )

                self.__cnt_id += 1
                pass
            pass
        pass

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

    def get_observers(self):
        return self.observers

    def print_status(self):
        for name, status in self.get_status().items():
            print("- sensor: [" + str(name) + "]:" + str(status.name))