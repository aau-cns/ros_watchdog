#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.

import os
import time
import ast
import configparser
import yaml
import rosgraph
import rospy
import math
from enum import Enum

class TopicStatus(Enum):    # RosWatchdog.status
    OK = 0                    # -> OK
    STARTING = 1            # -> OK
    ERROR = 2                 # -> ABORT
    UNOBSERVED= 3


class TopicActions(Enum):    # RosWatchdog.status
    NONE = 0                    # -> OK
    WARNING = 1                 # -> OK
    ERROR = 2                   # -> ABORT
    RESTART_ROSNODE= 3          # -> HOLD
    RESTART_SENSOR = 4          # -> HOLD


class TopicObserver(object):
    def __init__(self, topic_name, rate=1,
                 watchdog_action=0, timeout=0.0,
                 sensor_name='', node_name='', window_size=10, verbose=True):
        self.name = topic_name
        self.rate = float(rate)
        self.action = int(watchdog_action)
        self.sensor_name = sensor_name
        self.node_name = node_name
        self.timeout = float(timeout)
        self.window_size = int(window_size)
        if self.timeout == 0:
            self.timeout = self.window_size/self.rate if self.rate > 0.0 else 0.0

        self.bVerbose = verbose

        self.times = []
        self.msg_t0 = -1
        self.msg_tn = -1
        self.time_operational = -1

        self.topic = rosgraph.names.script_resolve_name('rostopic', self.name)
        self.status = TopicStatus.UNOBSERVED
        self.sub = None
        pass

    def stop_observation(self):
        self.status = TopicStatus.UNOBSERVED
        self.time_operational = -1
        if self.sub:
            self.sub.unregister()
            self.sub = None
            pass
        pass

    # important for high level logic to define when observation should start!
    def start_observation(self):
        self.times = []
        self.msg_t0 = -1
        self.msg_tn = -1
        self.time_operational = rospy.get_rostime().to_sec() + self.timeout
        self.status = TopicStatus.STARTING
        self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.callback_hz)
        pass

    def get_times(self):
        return self.times

    def set_times(self, value):
        self.times = value

    def callback_hz(self, data):
        """
        ros sub callback
        """
        curr = rospy.get_rostime().to_sec()

        if self.msg_tn < 0 or self.msg_t0 < 0:
            self.msg_t0 = curr
            self.msg_tn = curr
            self.times = []
        else:
            self.times.append(curr - self.msg_tn)
            self.msg_tn = curr
        if len(self.times) > self.window_size - 1:
            self.times.pop(0)

    def get_hz(self):
        n = len(self.times)
        # rate = (n - 1) / (rospy.get_time() - self.msg_t0)
        mean = sum(self.times) / n if n > 0 else 0
        rate = 1. / mean if mean > 0.0 else 0

        # std dev
        std_dev = math.sqrt(sum((x - mean) ** 2 for x in self.times) / n) if n > 0 else 0

        # min and max
        max_delta = max(self.times) if n > 0 else 0
        min_delta = min(self.times) if n > 0 else 0

        return rate, mean, std_dev, max_delta, min_delta

    def get_status(self):
        if self.status == TopicStatus.UNOBSERVED:
            return TopicStatus.UNOBSERVED

        t_curr = rospy.get_rostime().to_sec()

        # check if this topic is in specified dead time.
        if t_curr < (self.time_operational):
            if self.bVerbose:
                print("*  [" + self.name + "] still within initial timeout...")
            return TopicStatus.STARTING

        if self.msg_t0 < 0:
            if self.bVerbose:
                print("*  [" + self.name + "] no message received yet...")
            return TopicStatus.ERROR

        [rate, mean, std_dev, max_delta, min_delta] = self.get_hz()
        if rate < self.rate:
            if self.bVerbose:
                print("*  [" + self.name + "] expected rate " + str(self.rate) + " not reached: " + str(rate))
                print("*  -  stat:" + str([rate, mean, std_dev, max_delta, min_delta]))
            return TopicStatus.ERROR
        else:
            if self.bVerbose:
                print("*  [" + self.name + "] expected rate " + str(self.rate) + " reached: " + str(rate))
                print("*  -  stat:" + str([rate, mean, std_dev, max_delta, min_delta]))
            return TopicStatus.OK


    def get_action(self):
        return self.action

class TopicsObserver(object):
    """Node example class."""

    def __init__(self, topics_cfg_file, verbose=True):
        assert (os.path.exists(topics_cfg_file))
        self.bVerbose = verbose

        self.observers = {}
        config = configparser.ConfigParser()
        config.sections()
        config.read(topics_cfg_file)
        self.items = config.items()

        # the first element is default section!
        if len(self.items) < 2:
            rospy.logerr("ERROR: no topic objects in " + str(topics_cfg_file))

        for key, section in self.items:
            if key != 'DEFAULT':
                if self.bVerbose:
                    print(key)
                # read configuration:
                self.observers[key] = TopicObserver(
                                        topic_name=key,
                                        rate=float(section.get('rate', 1.0)),
                                        watchdog_action=int(section.get('watchdog_action', 0)),
                                        timeout=float(section.get('timeout', 0.0)),
                                        sensor_name=str(section.get('sensor_name', '')),
                                        node_name=str(section.get('node_name', '')),
                                        verbose=verbose)

    def exists(self, name):
        return self.observers.has_key(name)

    def start_observation(self):
        for key, val in self.observers.items():
            val.start_observation()

    def stop_observation(self):
        for key, val in self.observers.items():
            val.stop_observation()

    def get_status(self):
        status = {}
        for key, val in self.observers.items():
            status[key] = val.get_status()

        return status

    def print_status(self):
        for name, status in self.get_status().items():
            print("- topic:  [" + str(name) + "]:" + str(status.name))

    def get_action(self, name):
        if self.exists(name):
            return self.observers[name].get_action()
        else:
            return 0

    def has_node(self, node_name):
        topic_names = []
        for key, val in self.observers.items():
            if node_name == val.node_name:
                topic_names.append(key)
        return topic_names



if __name__ == '__main__':

    rospy.init_node("TopicsObserver")
    # Go to class functions that do all the heavy lifting.
    try:
        obs = TopicsObserver('topics.ini', verbose=False)

        obs.start_observation()
        while not rospy.is_shutdown():
            status = obs.get_status()
            print(str(status))

            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()
