#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.
import rospy
import os

from autonomy_msgs.msg import SystemStatus
from enum import Enum


from TopicsObserver import TopicsObserver
from Sensors import Sensors
from ros_watchdog.srv import status as StatusSrv
from ros_watchdog.srv import statusResponse as StatusSrvResp

class WatchdogActions(Enum):    # RosWatchdog.status
    NONE = 0                    # -> OK
    WARNING = 1                 # -> OK
    ERROR = 2                   # -> ABORT
    RESTART_ROSNODE= 3          # -> HOLD
    RESTART_SENSOR = 4          # -> HOLD



class RosWatchdog(object):
    """Node example class."""

    def __init__(self):
        # ROS parameters
        self.topics_cfg_file = rospy.get_param("~topics_cfg_file", "topics.ini")
        self.sensors_cfg_file = rospy.get_param("~sensors_cfg_file", "sensors.ini")
        self.topic_check_rate = rospy.get_param("~topic_check_rate", 1.0)
        self.bVerbose = rospy.get_param("~bVerbose",True)

        # topic observers
        self.topic_observer = TopicsObserver(self.topics_cfg_file)
        self.sensors = Sensors(self.sensors_cfg_file)

        self.status = SystemStatus.ABORT
        self.pub_status = rospy.Publisher("/status", SystemStatus, queue_size=10)
        self.set_status(SystemStatus.HOLD)
        self.do_check_topics = False

        # Declare our service object
        self.custom_srv = rospy.Service('/status_service', StatusSrv, self.handle_status_service)

    def is_node_running(self, node_name):
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n", "")

        if self.bVerbose:
            rospy.loginfo("existing nodes: " + str(nodes))

        if any(x == node_name for x in nodes):
            return True
        else:
            return False


    def start(self):
        self.topic_observer.start()
        self.do_check_topics = True

    def stop(self):
        self.do_check_topics = False

    def handle_status_service(self, req):
        """Create a handle for the custom service."""
        if self.bVerbose:
            rospy.loginfo("Received status request from " + str(req.source))

        resp = StatusSrvResp()
        resp.status = self.get_status_msg()
        return resp

    def check_topics(self):
        """Call at a specified interval to publish message."""
        if self.bVerbose:
            rospy.loginfo('check_topics')


        violation_detected = False
        for name, topic in self.topic_observer.topic_observers.items():
            if topic.is_violated(verbose=self.bVerbose):
                violation_detected = True

                action = WatchdogActions(topic.action)

                if action == WatchdogActions.NONE:
                    pass
                elif action == WatchdogActions.WARNING:
                    rospy.logwarn("WARNING: topic [" + str(name) + "] check failed! ")

                elif action == WatchdogActions.ERROR:
                    rospy.logerr("ERROR: topic [" + str(name) + "] check failed! ")
                    self.set_status(SystemStatus.ABORT, "topic check failed")

                elif action == WatchdogActions.RESTART_ROSNODE:
                    rospy.logwarn("WARNING: restarting node of [" + str(name) + "] " + str(topic.node_name))


                    if self.is_node_running(topic.node_name):
                        self.set_status(SystemStatus.HOLD, "restarting node of [" + str(name) + "] ")
                        if self.bVerbose:
                            rospy.loginfo("-- rosnode kill "+ topic.node_name)

                        os.system("rosnode kill "+ topic.node_name)
                    else:
                        rospy.logwarn("-- node ["+ topic.node_name + "] is not running!")
                        self.set_status(SystemStatus.ABORT, "node is not running")


                elif action == WatchdogActions.RESTART_SENSOR:
                    rospy.logwarn("WARNING: restarting sensor of [" + str(name) + "] " + str(topic.sensor_name))

                    if self.sensors.exists(topic.sensor_name):
                        if self.bVerbose:
                            print('  - sensor found!')
                        sensor = self.sensors.sensors[topic.sensor_name]
                        if self.bVerbose:
                            print('  - run:' + str(sensor.restart_script))

                        success = True
                        # TODO: implement synchronous script execution
                        #
                        #
                        #
                        #

                        if success:
                            self.set_status(SystemStatus.HOLD, "restarting sensor")
                        else:
                            if self.bVerbose:
                                print('  - ERROR: was not able to restart sensor!')
                            self.set_status(SystemStatus.ABORT, "sensor not restartable")

                    else:
                        if self.bVerbose:
                            print('  - ERROR: sensor not found!')
                        self.set_status(SystemStatus.ABORT, "sensor not found -> invalid configuration")

                else:
                    rospy.logerr('unknown action: ' + str(action))

                topic.reset()

        if not violation_detected:
            self.set_status(SystemStatus.OK)

    def set_status(self, status_, info_ = ""):
        if  status_ is not self.status:
            rospy.loginfo('status changed: ' + str(status_))
            self.status = status_

            msg = self.get_status_msg()
            msg.info = info_

            self.pub_status.publish(msg)

    def get_status_msg(self):
        msg = SystemStatus()
        msg.stamp = rospy.get_rostime().now()
        msg.status = self.status
        msg.source = "ros_watchdog"
        return msg

    def run(self):
        rate = rospy.Rate(self.topic_check_rate)
        cnt = 0
        while not rospy.is_shutdown():
            if self.do_check_topics:
                self.check_topics()
            elif cnt % 100 == 0:
                hello_str = "heart beat %s" % rospy.get_time()
                rospy.loginfo(hello_str)

            cnt = cnt + 1
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
