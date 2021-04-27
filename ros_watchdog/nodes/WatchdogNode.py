#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import rospy
import os
import typing as typ

from watchdog_msgs.msg import SystemStatus
from watchdog_msgs.msg import SystemStatusStamped
from watchdog_msgs.srv import Start as StartService
from watchdog_msgs.srv import StartRequest as StartReq
from watchdog_msgs.srv import StartResponse as StartRes

from watchdog.Watchdog import Watchdog


class WatchdogNode(object):

    def __init__(self):

        # ROS parameters
        self.topics_cfg_file = rospy.get_param("~topics_cfg_file", "topics.ini")
        self.sensors_cfg_file = rospy.get_param("~sensors_cfg_file", "sensors.ini")
        self.nodes_cfg_file = rospy.get_param("~nodes_cfg_file", "nodes.ini")
        self.topic_check_rate = rospy.get_param("~topic_check_rate", 1.0)
        self.bVerbose = rospy.get_param("~bVerbose", True)
        self.bUseStartupTO = rospy.get_param("~bUseStartupTO", True)

        # declare publishers
        self.pub_status = rospy.Publisher("/watchdog/status", SystemStatusStamped, queue_size=10)
        self.pub_log = rospy.Publisher("/watchdog/log", SystemStatusStamped, queue_size=10)

        # declare services
        self.start_srv = rospy.Service('service/start', StartService, self.handle_start_service)

        # create watchdog
        self.watchdog = Watchdog(verbose=self.bVerbose)

        # debug info
        rospy.loginfo("Started ROS Watchdog Node")
        pass  # def __init__

    def handle_start_service(self,
                             req,       # type: StartReq
                             ):
        # type: (...) -> StartRes

        # debug
        if self.bVerbose:
            rospy.loginfo("Received startup request from %s" % req.header.frame_id)
            pass

        # initialize watchdog
        config_files = {
            Watchdog.ObserverKeys.TOPIC: self.topics_cfg_file
        }
        self.watchdog.init(config_files)

        # start watchdog
        self.watchdog.start(self.bUseStartupTO)

        # wait required time for system checks
        rospy.loginfo("collecting startup data for %f seconds" % req.startup_time)
        rospy.sleep(req.startup_time)

        # return response
        res = StartRes()
        res.header.stamp = rospy.get_rostime().now()
        res.header.frame_id = "ros_watchdog"

        res.status.id = 0
        res.status.name = "global"
        res.status.status, res.status.info = self.watchdog.get_status_global()
        res.successful = True if res.status.status == 1 else False

        return res
        pass

    pass  # class WatchdogNode()


if __name__ == "__main__":
    rospy.init_node("ros_watchdog", log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting.
    try:
        node = WatchdogNode()

    except rospy.ROSInterruptException:
        pass

    # Allow ROS to go to all callbacks.
    rospy.spin()
    pass  # if __name__ == '__main__'
