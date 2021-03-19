#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.
import rospy
import os

from autonomy_msgs.msg import SystemStatus
from enum import Enum


from Observer import Observer
from NodesObserver import NodeStatus
from TopicsObserver import TopicStatus
from SensorsObserver import SensorStatus
from ros_watchdog.srv import status as StatusSrv
from ros_watchdog.srv import statusResponse as StatusSrvResp
from ros_watchdog.srv import wdstart as WdstartSrv
from ros_watchdog.srv import wdstartResponse as WdstartSrvResp


class RosWatchdog(object):
    """Node example class."""

    def __init__(self):
        # ROS parameters
        self.topics_cfg_file = rospy.get_param("~topics_cfg_file", "topics.ini")
        self.sensors_cfg_file = rospy.get_param("~sensors_cfg_file", "sensors.ini")
        self.nodes_cfg_file = rospy.get_param("~nodes_cfg_file", "nodes.ini")
        self.topic_check_rate = rospy.get_param("~topic_check_rate", 1.0)
        self.bVerbose = rospy.get_param("~bVerbose",True)

        # topic observers
        self.observer = Observer(topics_cfg_file=self.topics_cfg_file,
                                 nodes_cfg_file=self.nodes_cfg_file,
                                 sensors_cfg_file=self.sensors_cfg_file,
                                 verbose=self.bVerbose)

        self.status = SystemStatus.ABORT
        self.pub_status = rospy.Publisher("/status", SystemStatus, queue_size=10)
        self.set_status(SystemStatus.HOLD)
        self.state = "STOPPED"

        # Declare our service object
        self.custom_srv = rospy.Service('/status_service', StatusSrv, self.handle_status_service)
        self.startup_srv = rospy.Service('/start_service', WdstartSrv, self.handle_startup_service)
        pass

    def start(self):
        self.observer.start_observation()
        self.state = "STARTED"

    def stop(self):
        self.observer.stop_observation()
        self.state = "STOPPED"

    def handle_status_service(self, req):
        """Create a handle for the custom service."""
        if self.bVerbose:
            rospy.loginfo("Received status request from " + str(req.source))
            pass

        resp = StatusSrvResp()
        resp.status = self.get_status_msg()
        return resp

    def handle_startup_service(self, req):

        if self.bVerbose:
            rospy.loginfo("Received startup request")
            pass

        # start watchdog
        self.start()

        # wait required time for system checks
        rospy.loginfo("Collecting startup data for %f seconds" % req.startup_time)
        rospy.sleep(req.startup_time)

        # return response
        resp = WdstartSrvResp()
        resp.status = self.get_status_msg()
        resp.successful = True if resp.status.status < 2 else False
        return resp

    def check_topics(self):
        pass

    def set_status(self, status_, info_ = ""):
        if status_ is not self.status:
            rospy.loginfo('status changed: ' + str(status_))
            self.status = status_

            self.do_pub_status(info_=info_)
            pass
        pass

    def do_pub_status(self, info_=""):
        msg = self.get_status_msg()
        msg.info = info_
        self.pub_status.publish(msg)
        pass

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
            if self.state == "STARTED":
                nodes_status, topics_status, sensors_status = self.observer.process()

                if any(val == NodeStatus.ERROR for key, val in nodes_status.items()) or any(val == SensorStatus.ERROR for key, val in sensors_status.items()):
                    self.set_status(SystemStatus.ABORT)
                    if self.bVerbose:
                        rospy.logerr("- SystemStatus.ABORT")
                elif any(val == NodeStatus.STARTING for key, val in nodes_status.items())  or any(val == TopicStatus.STARTING for key, val in topics_status.items()):
                    self.set_status(SystemStatus.HOLD)
                    if self.bVerbose:
                        rospy.logwarn("- SystemStatus.HOLD")
                else:
                    self.set_status(SystemStatus.OK)
                    if self.bVerbose:
                        rospy.loginfo("- SystemStatus.OK")
                        pass
                    pass

                # publish status
                self.do_pub_status("heartbeat")

            elif cnt % 100 == 0:
                hello_str = "heart beat %s" % rospy.get_time()
                rospy.loginfo(hello_str)

            cnt = cnt + 1
            rate.sleep()
            pass
        pass
    pass


if __name__ == '__main__':
    rospy.init_node("ros_watchdog")
    # Go to class functions that do all the heavy lifting.
    try:
        obj = RosWatchdog()
        # obj.start()
        obj.run()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()
