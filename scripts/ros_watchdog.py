#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.

import rosgraph
import rospy
from std_msgs.msg import String

from scripts.SytemObserver import TopicsObserver

class RosWatchdog(object):
    """Node example class."""

    def __init__(self):
        self.topics_cfg_file = rospy.get_param("topics_cfg_file", "topics.ini")
        self.sensors_cfg_file = rospy.get_param("sensors_cfg_file", "sensors.ini")

        # Create a publisher for our custom message.
        self.pub = rospy.Publisher("info", String, queue_size=10)

        # Create a timer to go to a callback at a specified interval.
        rate = 1
        rospy.Timer(rospy.Duration(1.0 / rate), self.timer_cb)
        pass

    def timer_cb(self, _event):
        """Call at a specified interval to publish message."""
        print('timer_cb')

    def run(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.pub.publish(hello_str)
            rate.sleep()

if __name__ == '__main__':

    rospy.init_node("ros_watchdog")
    # Go to class functions that do all the heavy lifting.
    try:
        obj = RosWatchdog()
        obj.run()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()