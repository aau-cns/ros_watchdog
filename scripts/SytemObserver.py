#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.

import ast
import configparser
import yaml
import rosgraph
import rospy
import math
from enum import Enum


class RepairAction(Enum):
    RESTART= 1
    WARNING = 2
    ERROR = 3
    NONE = 4


class TopicObserver(object):
    def __init__(self, topic_name, rate, action_level, window_size=10):
        self.name = topic_name
        self.rate = rate
        self.action = RepairAction(action_level)
        self.times = []
        self.msg_t0 = -1
        self.msg_tn = 0
        self.window_size = window_size

        # TODO subsrcibe to the topic and create a statistic
        pass


    def get_times(self):
        return self.times

    def set_times(self, value):
        self.times = value

    def callback_hz(self, m, topic=None):
        """
        ros sub callback
        :param m: Message instance
        :param topic: Topic name
        """
        curr_rostime = rospy.get_rostime()
        curr = curr_rostime.to_sec()

        self.msg_tn = 0

        if self.msg_t0 < 0 or self.msg_t0 > curr:
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
        mean = sum(self.times) / n
        rate = 1. / mean if mean > 0. else 0

        # std dev
        std_dev = math.sqrt(sum((x - mean) ** 2 for x in self.times) / n)

        # min and max
        max_delta = max(self.times)
        min_delta = min(self.times)


class SystemObserver(object):
    """Node example class."""

    def __init__(self, config_filename):
        config = configparser.ConfigParser()
        config.sections()
        config.read(config_filename)
        self.items = config.items()
        self.start_time = rospy.get_time()
        self.topic_observers = {}

        pass

    def start(self):
        self.start_time = rospy.get_time()
        print(self.start_time)
        for key, section in self.items:
            if key != 'DEFAULT':
                print(key)



if __name__ == '__main__':

    rospy.init_node("SystemObserver")
    # Go to class functions that do all the heavy lifting.
    try:
        obs = SystemObserver('topics.ini')

        obs.start()

    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()
