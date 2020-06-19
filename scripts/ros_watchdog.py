#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.

import rosgraph
import rospy
from std_msgs.msg import String
from enum import Enum
import configparser

from scripts.SytemObserver import TopicsObserver

class WatchdogActions(Enum):    # RosWatchdog.status
    NONE = 0                    # -> OK
    WARNING = 1                 # -> OK
    ERROR = 2                   # -> LAND
    RESTART_ROSNODE= 3          # -> HOLD
    RESTART_SENSOR = 4          # -> HOLD



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

class RosWatchdog(object):
    """Node example class."""

    def __init__(self):
        # ROS parameters
        self.topics_cfg_file = rospy.get_param("topics_cfg_file", "topics.ini")
        self.sensors_cfg_file = rospy.get_param("sensors_cfg_file", "sensors.ini")
        self.topic_check_rate = rospy.get_param("topic_check_rate", 1.0)

        # topic observers
        self.topic_observer = TopicsObserver(self.topics_cfg_file)
        self.sensors = Sensors(self.sensors_cfg_file)

        self.status = "OK"
        self.pub_status = rospy.Publisher("status", String, queue_size=10)



    def start(self):
        self.topic_observer.start()
        # Create a timer to go to a callback at a specified interval.
        rate = 1
        rospy.Timer(rospy.Duration(1.0 / self.topic_check_rate), self.topic_check_cb)

    def topic_check_cb(self, _event):
        """Call at a specified interval to publish message."""
        rospy.loginfo('topic_check_cb')


        violation_detected = False
        for name, topic in self.topic_observer.topic_observers.items():
            if topic.is_violated():
                violation_detected = True

                action = WatchdogActions(topic.action)

                if action == WatchdogActions.NONE:
                    pass
                elif action == WatchdogActions.WARNING:
                    rospy.logwarn("WARNING: topic [" + str(name) + "] check failed! ")
                    pass
                elif action == WatchdogActions.ERROR:
                    rospy.logerr("ERROR: topic [" + str(name) + "] check failed! ")
                    self.set_status("LAND")
                    pass
                elif action == WatchdogActions.RESTART_ROSNODE:
                    rospy.logwarn("WARNING: restarting node of [" + str(name) + "] " + str(topic.node_name))
                    self.set_status("HOLD")
                    pass
                elif action == WatchdogActions.RESTART_SENSOR:
                    rospy.logwarn("WARNING: restarting sensor of [" + str(name) + "] " + str(topic.sensor_name))

                    if self.sensors.exists(topic.sensor_name):
                        print('  - sensor found!')
                        sensor = self.sensors.sensors[topic.sensor_name]
                        print('  - run:' + str(sensor.restart_script))
                    else:
                        print('  - ERROR: sensor not found!')

                    self.set_status("HOLD")
                    pass
                else:
                    rospy.logerr('unknown action: ' + str(action))

                topic.reset()

        if not violation_detected:
            self.set_status("OK")

    def set_status(self, status_):
        if  status_ is not self.status:
            rospy.loginfo('status changed')
            self.status = status_
            self.pub_status.publish(self.status)

    def run(self):
        rate = rospy.Rate(0.1)  # 10hz
        while not rospy.is_shutdown():
            hello_str = "heart beat %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            rate.sleep()

if __name__ == '__main__':

    rospy.init_node("ros_watchdog")
    # Go to class functions that do all the heavy lifting.
    try:
        obj = RosWatchdog()
        obj.start()
        obj.run()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()