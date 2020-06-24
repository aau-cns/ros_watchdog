#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import rospy
import os
import time
from enum import Enum
import configparser

class NodeStatus(Enum):    # RosWatchdog.status
    OK = 0                    # -> OK
    RESTARTING = 1            # -> OK
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



class NodeObserver(object):
    def __init__(self, node_name, max_restart_attempts = 0, restart_timeout = 0, verbose = True):
        self.node_name = node_name
        self.num_restarts = 0
        self.max_restart_attempts = int(max_restart_attempts)
        self.restart_timeout = float(restart_timeout)
        self.bVerbose = verbose
        self.stop_observation()

    def stop_observation(self):
        self.t0 = -1
        self.t_running = -1

    # important for high level logic to define when observation should start!
    def start_observation(self):
        self.t0 = rospy.get_rostime().now().to_sec()
        self.t_running = self.t0 + self.restart_timeout

    def kill_node(self):
        return kill_ros_node(self.node_name, self.bVerbose)

    def restart_node(self):

        if self.num_restarts < self.max_restart_attempts:
            if self.bVerbose:
                rospy.loginfo("-- restart node [" + self.node_name + "]")

            self.num_restarts += 1
            if self.kill_node():
                self.start_observation()
                return True
            else:
                return False
        elif self.num_restarts >= self.max_restart_attempts:
            self.num_restarts += 1
            rospy.logwarn("-- node [" + self.node_name + "] exceeded restart attempts!")

        return False

    def is_running(self):
        return is_ros_node_running(self.node_name, self.bVerbose)

    def get_status(self):
        # check if node observer was started
        # check if node is running otherwise restart
        # check num restart attempts
        # check if is in restart interval
        # otherwise OK.
        if self.t_running < 0 or self.t0 < 0:
            return NodeStatus.UNOBSERVED

        if self.num_restarts > self.max_restart_attempts:
            return NodeStatus.ERROR

        if not self.is_running():
            if not self.restart_node():
                return NodeStatus.ERROR


        t_curr = rospy.get_rostime().now().to_sec()
        if self.t_running > t_curr:
            return NodeStatus.RESTARTING  # all related TopicObserver need to restart their observation! as long as the node is restarting

        return NodeStatus.OK



class NodesObserver(object):
    def __init__(self, nodes_cfg_file, verbose = True):
        assert(os.path.exists(nodes_cfg_file))
        self.bVerbose = verbose

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
                                        max_restart_attempts=int(section.get('max_restart_attempts', 0)),
                                        restart_timeout=float(section.get('restart_timeout', 0)),
                                        verbose=self.bVerbose)

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

    def print_status(self):
        for name, status in self.get_status().items():
            print("- node:   [" + str(name) + "]:" + str(status.name))

if __name__ == '__main__':

    rospy.init_node("NodesObserver")
    # Go to class functions that do all the heavy lifting.
    try:
        obs = NodesObserver('nodes.ini')

        obs.start_observation()
        while not rospy.is_shutdown():
            status = obs.get_status()
            print(str(status))

            time.sleep(1)


    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()