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
from enum import Enum, unique
import typing as typ

from observer.Observer import Observer, ObserverStatus, ObserverSeverity


@unique
class TopicStatus(Enum):    # RosWatchdog.status
    NOMINAL = 0             # -> NOMINAL
    STARTING = 1            # -> NOMINAL
    NONCRITICAL = 2         # -> NON CRITICAL (message rate)
    ERROR = 3               # -> FAIL (depending on severity)
    UNOBSERVED = -1         # not started yet
    pass  # class TopicStatus(Enum)


@unique
class TopicActions(Enum):    # RosWatchdog.status
    NONE = 0                    # -> OK
    WARNING = 1                 # -> OK
    ERROR = 2                   # -> ABORT
    RESTART_ROSNODE = 3         # -> HOLD
    RESTART_SENSOR = 4          # -> HOLD
    pass


class TopicObserver(Observer):
    def __init__(self,
                 topic_name,            # type: str
                 observer_id,           # type: int
                 rate=1,                # type: typ.Union[float, str]
                 severity=0,            # type: typ.Union[int, str]
                 watchdog_action=0,     # type: typ.Union[int, str]
                 timeout=0.0,
                 sensor_name='',
                 node_name='',
                 window_size=10,
                 verbose=True,
                 rate_margin=0.1,
                 ):

        # initialize super
        super(TopicObserver, self).__init__(topic_name, observer_id, timeout, verbose)

        # set topic values
        self.rate = float(rate)
        self.severity = int(severity)
        self.action = int(watchdog_action)
        self.sensor_name = sensor_name
        self.node_name = node_name
        self.timeout = float(timeout)
        self.window_size = int(window_size)
        if self.timeout == 0:
            self.timeout = self.window_size/self.rate if self.rate > 0.0 else 0.0
            pass

        # set rate boundaries
        if not 0 < rate_margin <= 1:
            rospy.logwarn("Rate margin has to be between 0 and 1 (0-100%). Using standard of 0.1")
            rate_margin = 0.1
            pass
        self.__rate_margin = rate_margin * self.rate

        self.times = []
        self.msg_t0 = -1
        self.msg_tn = -1
        self.time_operational = -1

        self.topic = rosgraph.names.script_resolve_name('rostopic', self.name)
        self.sub = None
        pass

    def stop_observation(self):
        self.status = ObserverStatus.UNOBSERVED
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
        self.status = ObserverStatus.STARTING
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
        if self.status == ObserverStatus.UNOBSERVED:
            return ObserverStatus.UNOBSERVED

        t_curr = rospy.get_rostime().to_sec()

        # check if this topic is in specified dead time.
        if t_curr < self.time_operational:
            if self.do_verbose():
                print("*  [" + self.name + "] still within initial timeout...")
            return ObserverStatus.STARTING

        if self.msg_t0 < 0:
            if self.do_verbose():
                print("*  [" + self.name + "] no message received yet...")
            return ObserverStatus.ERROR

        [rate, mean, std_dev, max_delta, min_delta] = self.get_hz()

        # set status depending on rate
        if abs(rate - self.rate) > self.__rate_margin:
            # error condition
            if self.do_verbose():
                print("*  [" + self.name + "] expected rate " + str(self.rate) + " not reached: " + str(rate))
                print("*  -  stat:" + str([rate, mean, std_dev, max_delta, min_delta]))
                pass
            return ObserverStatus.ERROR

        if self.rate/20 < abs(rate - self.rate) < self.__rate_margin:
            # within boundaries for rate margin (accounting for numerical errors with rate/20)
            if self.do_verbose():
                print("*  [" + self.name + "] expected rate " + str(self.rate) + " slightly differs: " + str(rate))
                print("*  -  stat:" + str([rate, mean, std_dev, max_delta, min_delta]))
                pass
            return ObserverStatus.NONCRITICAL

        else:
            # nominal condition
            if self.do_verbose():
                print("*  [" + self.name + "] expected rate " + str(self.rate) + " reached: " + str(rate))
                print("*  -  stat:" + str([rate, mean, std_dev, max_delta, min_delta]))
                pass
            return ObserverStatus.NOMINAL

        pass  # def get_status()

    def get_severity(self):
        return self.severity

    def get_action(self):
        return self.action

class TopicsObserver(object):
    """Node example class."""

    def __init__(self,
                 topics_cfg_file,
                 verbose=True,
                 use_startup_to=True,
                 ):
        # do preliminary checks
        assert (os.path.exists(topics_cfg_file))
        self.bVerbose = verbose

        # setup ID counter
        self.__cnt_id = 1   # always start with 1, 0 --> global

        # read configs
        self.observers = {}
        config = configparser.ConfigParser()
        config.sections()
        config.read(topics_cfg_file)
        self.items = config.items()

        # the first element is default section!
        if len(self.items) < 2:
            rospy.logerr("ERROR: no topic objects in " + str(topics_cfg_file))
            pass

        # create dictionary of topic observers
        for key, section in self.items:
            if key != 'DEFAULT':
                if self.bVerbose:
                    print(key)
                    pass
                # check startup timeout
                timeout = float(section.get('timeout', '0.0')) if use_startup_to else 0.0

                # read configuration:
                self.observers[key] = TopicObserver(
                    topic_name=key,
                    observer_id=self.__cnt_id,
                    rate=float(section.get('rate', '1.0')),
                    rate_margin=float(section.get('margin', '0.1')),
                    severity=int(section.get('severity', '0')),
                    watchdog_action=int(section.get('watchdog_action', '0')),
                    timeout=timeout,
                    sensor_name=str(section.get('sensor_name', '')),
                    node_name=str(section.get('node_name', '')),
                    verbose=verbose,
                )

                self.__cnt_id += 1
                pass
            pass
        pass

    def exists(self, name):
        # type: (...) -> bool
        # return self.observers.has_key(name)
        # python 3 change
        return name in self.observers

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

    def get_observers(self):
        return self.observers

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
        obs = TopicsObserver('../../scripts/topics.ini', verbose=False)

        obs.start_observation()
        while not rospy.is_shutdown():
            status = obs.get_status()
            print(str(status))

            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()
