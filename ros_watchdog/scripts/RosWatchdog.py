#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.
import rospy
import os
import typing as typ

from watchdog_msgs.msg import SystemStatus
from watchdog_msgs.msg import SystemStatusStamped
from enum import Enum


from Observer import Observer
from NodesObserver import NodeStatus
from TopicsObserver import TopicStatus
from SensorsObserver import SensorStatus
from ros_watchdog.srv import status as StatusSrv
from ros_watchdog.srv import statusResponse as StatusSrvResp
from ros_watchdog.srv import wdstart as WdstartSrv
from ros_watchdog.srv import wdstartResponse as WdstartSrvResp
from ros_watchdog.srv import wderror as WdErrorSrv
from ros_watchdog.srv import wderrorRequest as WdErrorSrvRequ

from watchdog_msgs.srv import Start as StartService
from watchdog_msgs.srv import StartRequest as StartReq
from watchdog_msgs.srv import StartResponse as StartRes


class RosWatchdog(object):
    """Node example class."""

    class WatchdogStates(Enum):
        STOPPED = 0
        INITIALIZED = 1
        STARTED = 2
        pass  # class WatchdogStates(Enum)

    def __init__(self):

        # ROS parameters
        self.topics_cfg_file = rospy.get_param("~topics_cfg_file", "topics.ini")
        self.sensors_cfg_file = rospy.get_param("~sensors_cfg_file", "sensors.ini")
        self.nodes_cfg_file = rospy.get_param("~nodes_cfg_file", "nodes.ini")
        self.topic_check_rate = rospy.get_param("~topic_check_rate", 1.0)
        self.bVerbose = rospy.get_param("~bVerbose", True)
        self.bUseStartupTO = rospy.get_param("~bUseStartupTO", True)

        # topic observers
        self.observer = Observer(topics_cfg_file=self.topics_cfg_file,
                                 nodes_cfg_file=self.nodes_cfg_file,
                                 sensors_cfg_file=self.sensors_cfg_file,
                                 verbose=self.bVerbose,
                                 use_startup_to=self.bUseStartupTO,)

        # Declare Action Server on Autonomy side
        self.autonomy_action_req = None

        # Set status variables
        self.status = SystemStatus.ABORT
        self.pub_status = rospy.Publisher("status", SystemStatusStamped, queue_size=10)
        self.set_status(SystemStatus.HOLD)

        # setup state
        self.state = self.WatchdogStates.STOPPED

        # Declare our service object
        self.custom_srv = rospy.Service('/status_service', StatusSrv, self.handle_status_service)

        # declare services
        self.start_srv = rospy.Service('service/start', StartService, self.handle_start_service)

        # declare counters
        self.__cntStatus = 0      # type: int

        pass

    def start(self):
        # type: (...) -> None
        """Starts the watchdog node"""

        # check current state of watchdog
        if self.state is not self.WatchdogStates.STOPPED:
            rospy.logerr("Watchdog already started (or is starting). Doing nothing.")
            return
        else:
            self.state = self.WatchdogStates.INITIALIZED
            pass

        # start observer
        self.observer.start_observation()

        # connect to autonomy service
        # TODO(scm): check if still required
        if self.autonomy_action_req is None:
            rospy.wait_for_service('/autonomy_action')
            self.autonomy_action_req = rospy.ServiceProxy('/autonomy_action', WdErrorSrv, persistent=True)
            pass

        # set status to started
        self.state = self.WatchdogStates.STARTED
        pass  # def start()

    def stop(self):
        # type: (...) -> None
        """Stops the watchdog node"""

        # stop observations and set state
        self.observer.stop_observation()
        self.state = self.WatchdogStates.STOPPED
        pass  # def stop()

    def handle_status_service(self, req):
        """Create a handle for the custom service."""
        if self.bVerbose:
            rospy.loginfo("Received status request from " + str(req.source))
            pass

        resp = StatusSrvResp()
        resp.status = self.get_status_msg()
        return resp

    def handle_start_service(self,
                             req,       # type: StartReq
                             ):
        # type: (...) -> StartRes

        # debug
        if self.bVerbose:
            rospy.loginfo("Received startup request from %s" % req.header.frame_id)
            pass

        # start watchdog
        self.start()

        # wait required time for system checks
        rospy.loginfo("collecting startup data for %f seconds" % req.startup_time)
        rospy.sleep(req.startup_time)

        # return response
        res = StartRes()
        res.header.stamp = rospy.get_rostime().now()
        res.header.frame_id = "ros_watchdog"
        res.status = self.get_status_msg()
        res.successful = True if res.status.status < 4 else False
        return res

    def check_topics(self):
        pass

    def set_status(self, status_, info_=""):
        if status_ is not self.status:
            rospy.loginfo('status changed: ' + str(status_))
            self.status = status_

            # call service to publish change in status
            self.do_service_status()
            pass
        pass

    def do_pub_status(self, info_=""):
        """Publishes the current system status"""
        # setup msg
        msg = self.get_status_stamped_msg()
        msg.header.seq = self.__cntStatus
        msg.status.info = info_

        # publish msg
        self.pub_status.publish(msg)
        pass

    def do_service_status(self):
        if self.bVerbose:
            rospy.loginfo("Calling autonomy service ...")
            pass

        if self.autonomy_action_req is not None:
            try:
                msg = WdErrorSrvRequ()
                msg.status = self.get_status_msg()
                resp = self.autonomy_action_req(msg)

                if self.bVerbose:
                    rospy.loginfo("... got response %d" % resp.received)
                    pass

            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)
                pass
            pass
        else:
            rospy.logerr("Do not have any connection to autonomy service")
        pass

    def get_status_stamped_msg(self):
        # type: (...) -> SystemStatusStamped
        """Returns the system status as a stamped ROS message"""

        msg = SystemStatusStamped()
        msg.header.stamp = rospy.get_rostime().now()
        msg.header.frame_id = "ros_watchdog"
        msg.status = self.get_status_msg
        return msg

    def get_status_msg(self):
        # type: (...) -> SystemStatus
        """Returns the system status as a ROS message"""

        msg = SystemStatus()
        msg.id = 0  # global ID = 0
        msg.name = "global"
        msg.status = self.status
        return msg

    def run(self):
        rate = rospy.Rate(self.topic_check_rate)
        cnt = 0

        # main thread
        while not rospy.is_shutdown():
            # check if started
            if self.state == self.WatchdogStates.STARTED:
                nodes_status, topics_status, sensors_status = self.observer.process()

                # check for errors
                if any(val == NodeStatus.ERROR for key, val in nodes_status.items()) \
                        or any(val == SensorStatus.ERROR for key, val in sensors_status.items()):
                    self.set_status(SystemStatus.ABORT)
                    if self.bVerbose:
                        rospy.logerr("- SystemStatus.ABORT")
                        pass
                    pass

                # check for starting
                elif any(val == NodeStatus.STARTING for key, val in nodes_status.items()) \
                        or any(val == TopicStatus.STARTING for key, val in topics_status.items()):
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

            # sleep remainder of rate
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
    pass  # if __name__ == '__main__'
