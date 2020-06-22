#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import configparser


class Sensor(object):
    def __init__(self, name, restart_script=''):
        pass
        self.name = name
        self.restart_script = restart_script

    def restart(self):
        print('restart sensor ' + str(self.name))
        print(' -- running: ' + str(self.restart_script))


class Sensors(object):
    def __init__(self, sensors_cfg_file):
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
                                           restart_script=str(section.get('restart_script', '')))



    def restart(self):
        for key, val in self.sensors.items():
            val.restart()

    def exists(self, key):
        if key in self.sensors:
            return True
        return False
