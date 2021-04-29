#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import rospy
import os
import typing as typ

from watchdog_msgs.msg import SystemStatus, SystemStatusStamped
from watchdog_msgs.msg import StatusChangesArray, StatusChangesArrayStamped
from watchdog_msgs.srv import Start as StartService
from watchdog_msgs.srv import StartRequest as StartReq
from watchdog_msgs.srv import StartResponse as StartRes

from watchdog.Watchdog import Watchdog


class WatchdogNode(object):

    def __init__(self):

        # ROS parameters
        self.topics_cfg_file = rospy.get_param("~topics_cfg_file", "topics.ini")
        self.sensors_cfg_file = rospy.get_param("~sensors_cfg_file", "sensors.ini")
        self.drivers_cfg_file = rospy.get_param("~drivers_cfg_file", "sensors.ini")
        self.nodes_cfg_file = rospy.get_param("~nodes_cfg_file", "nodes.ini")
        self.topic_check_rate = rospy.get_param("~topic_check_rate", 1.0)
        self.bVerbose = rospy.get_param("~bVerbose", True)
        self.bUseStartupTO = rospy.get_param("~bUseStartupTO", True)

        # declare publishers
        self.pub_status = rospy.Publisher("/watchdog/status", SystemStatusStamped, queue_size=1)
        self.pub_log = rospy.Publisher("/watchdog/log", StatusChangesArrayStamped, queue_size=1)

        # declare services
        self.start_srv = rospy.Service(
            '/watchdog/service/start', StartService, self.handle_start_service)

        # declare counters
        self.__cntStatus = 0        # type: int
        self.__cntLog = 0           # type: int

        # create watchdog
        self.watchdog = Watchdog(verbose=self.bVerbose)

        # debug info
        rospy.loginfo("Started ROS Watchdog Node")
        pass  # def __init__

    def run(self):
        rate = rospy.Rate(self.topic_check_rate)
        cnt = 0

        # main thread
        while not rospy.is_shutdown():
            # flags
            do_pub_hb = False

            # check WD state
            wd_state = self.watchdog.get_state()
            rospy.logdebug("NODE == wd_state %s" % str(wd_state))
            if wd_state == Watchdog.States.RUNNING:
                # do watching
                self.watchdog.watch()

                # publish changes
                self.__publish_changes()

                # set publish hb
                do_pub_hb = True
                pass
            elif wd_state == Watchdog.States.STARTING:
                # set publish hb
                do_pub_hb = True
                pass
            else:
                pass

            # publish heartbeat status message
            if do_pub_hb:
                self.__publish_status()
                pass

            # sleep remainder of rate
            cnt += 1
            rate.sleep()

            pass  # while not rospy.is_shutdown()

        pass  # def run

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
            Watchdog.ObserverKeys.TOPIC: self.topics_cfg_file,
            Watchdog.ObserverKeys.DRIVER: self.drivers_cfg_file,
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

        res.status = self.__get_status_msg()
        res.successful = True if res.status.status == 1 else False

        return res
        pass

    def __publish_status(self):
        # create msg
        msg = SystemStatusStamped()
        msg.header.stamp = rospy.get_rostime().now()
        msg.header.frame_id = "ros_watchdog"
        msg.header.seq = self.__cntStatus
        msg.status = self.__get_status_msg()

        # publish msg
        self.pub_status.publish(msg)

        # increase counter
        self.__cntStatus += 1
        pass

    def __publish_changes(self):
        status_changes = self.watchdog.get_status_changes()

        # check if changes happend
        if len(status_changes) > 0:
            # change happend
            if self.bVerbose:
                rospy.logwarn("Status change detected")
                pass

            # get current status
            status_current = self.watchdog.get_status_all()

            # setup msg
            msg = StatusChangesArrayStamped()
            msg.header.stamp = rospy.get_rostime().now()
            msg.header.seq = self.__cntLog
            msg.header.frame_id = "ros_watchdog"

            # add all changes to msg
            for obskey, obs in status_changes.items():

                for assetkey, asset_status in obs.items():
                    status_msg = SystemStatus()
                    status_msg.id = self.watchdog.get_asset_id(obskey, assetkey)
                    status_msg.group = obskey.value
                    status_msg.name = str(assetkey)
                    status_msg.status = asset_status.value

                    msg.data.changes.append(status_msg)
                    pass
                pass

            # add all statuses to msg
            for obskey, obs in status_current.items():

                for assetkey, asset_status in obs.items():
                    status_msg = SystemStatus()
                    status_msg.id = self.watchdog.get_asset_id(obskey, assetkey)
                    status_msg.group = obskey.value
                    status_msg.name = str(assetkey)
                    status_msg.status = asset_status.value

                    msg.data.current.append(status_msg)
                    pass
                pass

            # publish message
            self.pub_log.publish(msg)

            # increase counter
            self.__cntLog += 1
            pass  # if len(status_changes) > 0

        pass  # def __publish_changes()

    ####################
    # GETTER
    ####################

    def __get_status_msg(self,
                         asset_id=0,
                         asset_name="global",
                         ):
        msg = SystemStatus()
        msg.id = asset_id
        msg.name = asset_name
        msg.status, msg.info = self.watchdog.get_status_global()
        return msg
        pass

    pass  # class WatchdogNode()


if __name__ == "__main__":
    # rospy.init_node("ros_watchdog", log_level=rospy.DEBUG)
    rospy.init_node("ros_watchdog", log_level=rospy.INFO)

    # Go to class functions that do all the heavy lifting.
    try:
        node = WatchdogNode()
        node.run()

    except rospy.ROSInterruptException:
        pass

    # Allow ROS to go to all callbacks.
    rospy.spin()
    pass  # if __name__ == '__main__'
