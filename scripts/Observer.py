#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.
import rospy
import os
import time

from autonomy_msgs.msg import SystemStatus
from enum import Enum


from TopicsObserver import TopicsObserver
from SensorsObserver import SensorsObserver
from NodesObserver import NodesObserver

class Observer(object):
    def __init__(self, topics_cfg_file, nodes_cfg_file, sensors_cfg_file, verbose=True):
        self.topcis_obs = TopicsObserver(topics_cfg_file=topics_cfg_file, verbose=verbose)
        self.nodes_obs = NodesObserver(nodes_cfg_file=nodes_cfg_file, verbose=verbose)
        self.sensors_obs = SensorsObserver(sensors_cfg_file=sensors_cfg_file)
        pass


    def get_status(self):
        return {}

    def start_observation(self):
        pass

    def stop_observation(self):
        pass

if __name__ == '__main__':

    rospy.init_node("Observer")
    # Go to class functions that do all the heavy lifting.
    try:
        obs = Observer('topics.ini', 'nodes.ini', 'sensors.ini', True)

        obs.start_observation()
        while not rospy.is_shutdown():
            status = obs.get_status()
            print(str(status))

            time.sleep(1)


    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()