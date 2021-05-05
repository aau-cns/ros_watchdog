#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.
import rospy
import os
import time
import typing as typ

from watchdog_msgs.msg import SystemStatusStamped
from enum import Enum


from TopicsObserver import TopicsObserver, TopicStatus, TopicActions
from DriversObserver import DriversObserver, DriverStatus
from NodesObserver import NodesObserver, NodeStatus


class SensorsObserver(object):
    def __init__(self,
                 topics_cfg_file,       # type: str
                 nodes_cfg_file,        # type: str
                 sensors_cfg_file,      # type: str
                 verbose=True,          # type: bool
                 use_startup_to=True,   # type: bool
                 ):

        # setup observers
        self.topics_obs = TopicsObserver(topics_cfg_file=topics_cfg_file, verbose=False, use_startup_to=use_startup_to)
        self.nodes_obs = NodesObserver(nodes_cfg_file=nodes_cfg_file, verbose=False, use_startup_to=use_startup_to)
        self.sensors_obs = DriversObserver(sensors_cfg_file=sensors_cfg_file)

        # setup flags
        self.bVerbose = verbose
        self.bUseStartupTO = use_startup_to

        pass  # def __init__(...)

    def start_observation(self):
        self.nodes_obs.start_observation()
        # topic observer will start automatically with node observer
        # self.topics_obs.start_observation()
        self.sensors_obs.start_observation()
        pass  # def start_observations()

    def stop_observation(self):
        self.nodes_obs.stop_observation()
        self.topics_obs.stop_observation()
        self.sensors_obs.stop_observation()
        pass  # def stop_observations()

    def process(self):

        nodes_status = self.check_nodes()
        topics_status = self.check_topics()
        sensors_status = self.check_sensors()
        return nodes_status, topics_status, sensors_status
        pass  # def process()

    def check_nodes(self):
        # type: (...) -> (...)
        """Checks the status of all nodes"""
        # modifies topic states
        if self.bVerbose:
            self.nodes_obs.print_status()

        nodes_status = self.nodes_obs.get_status()
        for node_name, node_status in nodes_status.items():
            if node_status == NodeStatus.STARTING or node_status == NodeStatus.UNOBSERVED:
                topics_names = self.topics_obs.has_node(node_name)
                for topic_name in topics_names:
                    if self.topics_obs.exists(topic_name):
                        self.topics_obs.observers[topic_name].stop_observation()

            if node_status == NodeStatus.ERROR:
                topics_names = self.topics_obs.has_node(node_name)
                for topic_name in topics_names:
                    if self.topics_obs.exists(topic_name):
                        self.topics_obs.observers[topic_name].stop_observation()

                        # no need to observe and check topic, as node is erroneous
                        self.topics_obs.observers[topic_name].status = TopicStatus.UNOBSERVED
                pass

            # start topic observer, once the node is operational
            if node_status == NodeStatus.NOMINAL:
                topics_names = self.topics_obs.has_node(node_name)
                for topic_name in topics_names:
                    if self.topics_obs.exists(topic_name):
                        if self.topics_obs.observers[topic_name].status == TopicStatus.UNOBSERVED:
                            self.topics_obs.observers[topic_name].start_observation()

        return nodes_status

    def check_topics(self):
        # type: (...) -> None
        """Checks the status of all observed topics"""

        # modifies node states
        if self.bVerbose:
            self.topics_obs.print_status()

        topics_status = self.topics_obs.get_status()

        for topic_name, topic_status in topics_status.items():
            if topic_status == TopicStatus.UNOBSERVED:
                pass

            elif topic_status == TopicStatus.ERROR:
                topic = self.topics_obs.observers[topic_name]
                action = TopicActions(topic.action)
                node_name = topic.node_name
                sensor_name = topic.driver_name

                if self.bVerbose:
                    print("*  [" + str(topic_name) + "]: ERROR.. ")

                if action == TopicActions.NONE:
                    print("*  - topic action -> none ...")
                    pass
                elif action == TopicActions.WARNING:
                    print("*  - topic action -> warning ...")
                    pass
                elif action == TopicActions.ERROR:
                    print("*  - topic action -> error ...")
                    pass
                elif action == TopicActions.RESTART_ROSNODE:
                    if self.nodes_obs.observers[node_name].restart_node():
                        print("*  - topic action -> drivers node SUCCESS")
                    else:
                        print("*  - topic action -> drivers node FAILURE")
                elif action == TopicActions.RESTART_SENSOR:
                    if self.sensors_obs.exists(sensor_name):
                        if not self.sensors_obs.observers[sensor_name].restart():
                            print("*    sensor " + str(sensor_name) + " restarting failed!")

                        if self.nodes_obs.observers[node_name].restart_node():
                            print("*  - topic action -> drivers node SUCCESS")
                        else:
                            print("*  - topic action -> drivers node FAILURE")
                    else:
                        print("*    sensor " + str(sensor_name) + " does not exist!")
                    print("*  - topic action -> drivers sensor...")
                else:
                    print("*  - unknown action")
                    assert(False)

            elif topic_status == TopicStatus.NOMINAL:
                pass


        return topics_status

    def check_sensors(self):
        # modifies node states
        if self.bVerbose:
            self.sensors_obs.print_status()

        return self.sensors_obs.get_status()


if __name__ == '__main__':

    rospy.init_node("Observer")
    # Go to class functions that do all the heavy lifting.
    try:
        obs = SensorsObserver('topics.ini', 'nodes.ini', 'sensors.ini', True)

        obs.start_observation()
        while not rospy.is_shutdown():
            status = obs.process()

            rospy.loginfo("Sleep...")
            time.sleep(1)
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()