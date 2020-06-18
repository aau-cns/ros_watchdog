#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.

import ast
import configparser
import yaml
import rosgraph
import rospy
from enum import Enum


class RepairAction(Enum):
    RESTART= 1
    WARNING = 2
    ERROR = 3
    NONE = 4


class TopicObserver(object):
    def __init__(self):
        self.name = "topic_name"
        self.rate = 10
        self.action = RepairAction.NONE
        pass


    def load_from_config(self, section):
        self.action = RepairAction(int(section.get('repair_action', 0)))
        entry = section.get('topic_list')
        self.topics_list = []
        if entry is not None:
            self.topics_list = ast.literal_eval(entry)

        self.rate = section.get('rate', 1)
        print('load from section')
        print(self.action)
        print(self.topics_list)


class NodeObserver(object):
    """Node example class."""

    def __init__(self, config_section):
        config_section = configparser.ConfigParser()


        self.startup_time = int(config_section.get('startup_time', '10'))
        self.launch_file = config_section.get('launch_file', 'default.launch')
        topic_list = ast.literal_eval(config_section.get("section", "topics"))


        print('load from section')
        print(str(self.startup_time))
        print(self.launch_file)

        pass