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

from observer.Observer import Observer, Observers, ObserverStatus, ObserverSeverity


# @unique
# class TopicStatus(Enum):    # RosWatchdog.status
#     NOMINAL = 0             # -> NOMINAL
#     STARTING = 1            # -> NOMINAL
#     NONCRITICAL = 2         # -> NON CRITICAL (message rate)
#     ERROR = 3               # -> FAIL (depending on severity)
#     UNOBSERVED = -1         # not started yet
#     pass  # class TopicStatus(Enum)
#
#
# @unique
# class TopicActions(Enum):    # RosWatchdog.status
#     NONE = 0                    # -> OK
#     WARNING = 1                 # -> OK
#     ERROR = 2                   # -> ABORT
#     RESTART_ROSNODE = 3         # -> HOLD
#     RESTART_SENSOR = 4          # -> HOLD
#     pass


class TopicObserver(Observer):
    def __init__(
            self,
            topic_name,                             # type: str
            entity_id,                              # type: str
            timeout=0.0,                            # type: typ.Union[float, str]
            rate=1,                                 # type: typ.Union[float, str]
            severity=0,                             # type: typ.Union[int, str]
            watchdog_action=0,                      # type: typ.Union[int, str]
            driver_name='',                         # type: str
            node_name='',                           # type: str
            window_size=10,                         # type: int
            rate_margin=0.1,                        # type: typ.Union[float, str]
            verbose=True,                           # type: bool
            ):

        # initialize super
        super(TopicObserver, self).__init__(topic_name, entity_id, float(timeout), verbose)

        # set topic values
        self.rate = float(rate)
        self.severity = int(severity)
        self.action = int(watchdog_action)
        self.driver_name = driver_name
        self.node_name = node_name
        self.timeout = float(timeout)
        self.window_size = int(window_size)
        if self.timeout == 0:
            self.timeout = self.window_size/self.rate if self.rate > 0.0 else 0.0
            pass

        # set rate boundaries
        rate_margin = float(rate_margin)
        if not 0 < rate_margin <= 1:
            rospy.logwarn("Rate margin has to be between 0 and 1 (0-100%). "
                          "Using standard value of 0.1 (10% of expected rate).")
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

    def update(self):
        if self.status == ObserverStatus.UNOBSERVED:
            return  # ObserverStatus.UNOBSERVED

        t_curr = rospy.get_rostime().to_sec()

        # check if this topic is in specified dead time.
        if t_curr < self.time_operational:
            if self.do_verbose():
                print("*  [" + self.name + "] still within initial timeout...")
                pass
            self.status = ObserverStatus.STARTING
            return

        if self.msg_t0 < 0:
            if self.do_verbose():
                print("*  [" + self.name + "] no message received yet...")
            self.status = ObserverStatus.ERROR
            return

        [rate, mean, std_dev, max_delta, min_delta] = self.get_hz()

        # set status depending on rate
        if self.rate - rate > self.__rate_margin:
            # error condition; rate not fullfilled
            if self.do_verbose():
                print("*  [" + self.name + "] expected rate " + str(self.rate) + " not reached: " + str(rate))
                print("*  -  stat:" + str([rate, mean, std_dev, max_delta, min_delta]))
                pass
            self.status = ObserverStatus.ERROR
            return

        if self.rate/100 < abs(rate - self.rate) < self.__rate_margin or rate - self.rate > self.__rate_margin:
            # within boundaries for rate margin (accounting for numerical errors with rate/20)
            # or rate is higher than expected
            if self.do_verbose():
                print("*  [" + self.name + "] expected rate " + str(self.rate) + " slightly differs: " + str(rate))
                print("*  -  stat:" + str([rate, mean, std_dev, max_delta, min_delta]))
                pass
            self.status = ObserverStatus.NONCRITICAL
            return

        else:
            # nominal condition
            if self.do_verbose():
                print("*  [" + self.name + "] expected rate " + str(self.rate) + " reached: " + str(rate))
                print("*  -  stat:" + str([rate, mean, std_dev, max_delta, min_delta]))
                pass
            self.status = ObserverStatus.NOMINAL
            return

        pass  # def update()

    def act(self, **kwargs):
        pass  # def act()

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

        # FIX(scm): no new messages, yet rated state the same (had to clear the times vec)
        # clear times array to account for no new messages (after X sec)
        self.times = []

        return rate, mean, std_dev, max_delta, min_delta

    def get_severity(self):
        return self.severity

    def get_action(self):
        return self.action

    pass  # class TopicObserver(...)


class TopicsObserver(Observers):
    """Node example class."""

    def __init__(self,
                 cfg_file,                          # type: str
                 verbose=True,                      # type: bool
                 use_startup_to=True,               # type: bool
                 ):
        # call super constructor
        super(TopicsObserver, self).__init__(
            cfg_file=cfg_file,
            verbose=verbose,
            name="TopicsObserver"
        )

        # setup ID counter
        self.__cnt_id = 1   # always start with 1, 0 --> global

        # create dictionary of topic observers
        for key, section in self.config_dict.items():
            # debugging
            if self.do_verbose():
                rospy.loginfo("Adding topic %s" % str(key))
                pass

            # check startup timeout
            timeout = float(section.get('timeout', '0.0')) if use_startup_to else 0.0

            # read configuration:
            self.observers[key] = TopicObserver(
                topic_name=key,
                entity_id=str(section.get('entity_id', 'undefined')),
                rate=float(section.get('rate', '1.0')),
                rate_margin=float(section.get('margin', '0.1')),
                severity=int(section.get('severity', '0')),
                watchdog_action=int(section.get('watchdog_action', '0')),
                timeout=timeout,
                driver_name=str(section.get('sensor_name', '')),
                node_name=str(section.get('node_name', '')),
                verbose=verbose,
            )

            self.__cnt_id += 1
            pass  # for key, section in self.config_dict.items()

        pass  # def __init__(...)

    # def exists(self, name):
    #     # type: (...) -> bool
    #     # return self.observers.has_key(name)
    #     # python 3 change
    #     return name in self.observers
    #
    # def start_observation(self):
    #     for key, val in self.observers.items():
    #         val.start_observation()
    #
    # def stop_observation(self):
    #     for key, val in self.observers.items():
    #         val.stop_observation()

    # def get_status(self):
    #     status = {}
    #     for key, val in self.observers.items():
    #         status[key] = val.get_status()
    #
    #     return status

    # def get_observers(self):
    #     return self.observers
    #
    # def print_status(self):
    #     for name, status in self.get_status().items():
    #         print("- topic:  [" + str(name) + "]:" + str(status.name))

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
