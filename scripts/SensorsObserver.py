#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import configparser
import os

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


    def clear_stats(self):
        self.num_restarts = 0


class SensorsObserver(object):
    def __init__(self, sensors_cfg_file):
        assert (os.path.exists(sensors_cfg_file))
        self.sensors = {}
        config = configparser.ConfigParser()
        config.sections()
        config.read(sensors_cfg_file)
        self.items = config.items()

        # the first element is default section!
        if len(self.items) < 2:
            print("ERROR: no sensors objects in " + str(sensors_cfg_file))

        for key, section in self.items:
            if key != 'DEFAULT':
                print(key)
                # read configuration:
                self.sensors[key] = Sensor(name=key,
                                           restart_script=str(section.get('restart_script', '')),
                                           restart_attempts=str(section.get('restart_attempts', 1))
                                           )



    def restart(self):
        for key, val in self.sensors.items():
            val.restart()

    def exists(self, key):
        if key in self.sensors:
            return True
        return False
