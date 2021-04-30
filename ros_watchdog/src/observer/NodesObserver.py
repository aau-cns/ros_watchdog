#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import rospy
import os
import time
from enum import Enum
import configparser
import typing as typ

from observer.Observer import Observer, ObserverStatus, ObserverSeverity

class NodeStatus(Enum):    # RosWatchdog.status
    OK = 0                    # -> OK
    STARTING = 1            # -> OK
    ERROR = 2                 # -> ABORT
    UNOBSERVED= 3


def is_ros_node_running(node_name, verbose):
    nodes = os.popen("rosnode list").readlines()
    for i in range(len(nodes)):
        nodes[i] = nodes[i].replace("\n", "")

    if verbose:
        rospy.loginfo("existing nodes: " + str(nodes))

    if any(x == node_name for x in nodes):
        return True
    else:
        return False


def kill_ros_node(node_name, verbose):
    if is_ros_node_running(node_name, verbose):
        if verbose:
            rospy.loginfo("-- rosnode kill " + node_name)

        os.system("rosnode kill " + node_name)
        return True
    else:
        rospy.logwarn("-- node [" + node_name + "] is not running!")

    return False


class NodeObserver(Observer):
    def __init__(
            self,
            node_name,                      # type: str
            entity_id,                      # type: str
            max_restart_attempts=0,         # type: typ.Union[str, int]
            restart_timeout=0.0,            # type: typ.Union[str, float]
            verbose=True,                   # type: bool
            ):

        # initialize super
        super(NodeObserver, self).__init__(str(node_name), entity_id, float(restart_timeout), verbose)

        self.num_restarts = 0
        self.max_restart_attempts = int(max_restart_attempts)
        self.restart_timeout = float(restart_timeout)
        self.stop_observation()
        pass  # def __init__(...)

    def stop_observation(self):
        self.t0 = -1
        self.t_running = -1

    # important for high level logic to define when observation should start!
    def start_observation(self):
        self.t0 = rospy.get_rostime().now().to_sec()
        self.t_running = self.t0 + self.restart_timeout

    def kill_node(self):
        return kill_ros_node(self.get_name(), self.do_verbose())

    def restart_node(self):

        if self.num_restarts < self.max_restart_attempts:
            if self.do_verbose():
                rospy.loginfo("-- drivers node [" + self.name + "]")

            self.num_restarts += 1
            if self.kill_node():
                self.start_observation()
                return True
            else:
                return False
        elif self.num_restarts >= self.max_restart_attempts:
            self.num_restarts += 1
            rospy.logwarn("-- node [" + self.name + "] exceeded drivers attempts!")

        return False

    def is_running(self):
        return is_ros_node_running(self.get_name(), self.do_verbose())

    def get_status(self):
        if self.t_running < 0 or self.t0 < 0:
            return ObserverStatus.UNOBSERVED

        if self.num_restarts > self.max_restart_attempts:
            return ObserverStatus.ERROR

        if not self.is_running():
            if not self.restart_node():
                return ObserverStatus.ERROR

        t_curr = rospy.get_rostime().now().to_sec()
        if self.t_running > t_curr:
            return ObserverStatus.STARTING  # all related TopicObserver need to drivers their observation! as long as the node is restarting

        return ObserverStatus.NOMINAL
        # return self.status

    # def update_status(self):
    #     # check if node observer was started
    #     # check if node is running otherwise drivers
    #     # check num drivers attempts
    #     # check if is in drivers interval
    #     # otherwise OK.
    #     if self.t_running < 0 or self.t0 < 0:
    #         self.status = ObserverStatus.UNOBSERVED
    #         return
    #
    #     if self.num_restarts > self.max_restart_attempts:
    #         self.status = ObserverStatus.ERROR
    #         return
    #
    #     if not self.is_running():
    #         if not self.restart_node():
    #             self.status = ObserverStatus.ERROR
    #             return
    #
    #     t_curr = rospy.get_rostime().now().to_sec()
    #     if self.t_running > t_curr:
    #         self.status = ObserverStatus.STARTING  # all related TopicObserver need to drivers their observation! as long as the node is restarting
    #         return
    #
    #     self.status = ObserverStatus.NOMINAL
    #     pass




class NodesObserver(object):
    def __init__(self, nodes_cfg_file, verbose=True, use_startup_to=True):
        assert(os.path.exists(nodes_cfg_file))
        self.bVerbose = verbose

        # setup ID counter
        self.__cnt_id = 1   # always start with 1, 0 --> global

        self.observers = {}
        config = configparser.ConfigParser()
        config.sections()
        config.read(nodes_cfg_file)
        self.items = config.items()

        # the first element is default section!
        if len(self.items) < 2:
            rospy.logerr("ERROR: no node objects in " + str(nodes_cfg_file))

        for key, section in self.items:
            if key != 'DEFAULT':
                if self.bVerbose:
                    print(key)
                # read configuration:
                self.observers[key] = NodeObserver(
                    node_name=key,
                    entity_id=str(section.get('entity_id', 'undefined')),
                    max_restart_attempts=int(section.get('max_restart_attempts', '0')),
                    restart_timeout=float(section.get('restart_timeout', '0.0')),
                    verbose=self.bVerbose
                )

                self.__cnt_id += 1
                pass
            pass
        pass

    def exists(self, node_name):
        return self.observers.has_key(node_name)

    def start_observation(self):
        for key, val in self.observers.items():
            val.start_observation()

    def stop_observation(self):
        for key, val in self.observers.items():
            val.stop_observation()

    def get_status(self):
        status = {}
        for key, val in self.observers.items():
            status[key] = val.get_status()

        return status

    def get_observers(self):
        return self.observers

    def print_status(self):
        for name, status in self.get_status().items():
            print("- node:   [" + str(name) + "]:" + str(status.name))

if __name__ == '__main__':

    rospy.init_node("NodesObserver")
    # Go to class functions that do all the heavy lifting.
    try:
        obs = NodesObserver('../../scripts/nodes.ini')

        obs.start_observation()
        while not rospy.is_shutdown():
            status = obs.get_status()
            print(str(status))

            time.sleep(1)


    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()