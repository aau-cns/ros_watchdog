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

from observer.Observer import ObserverStatus, ObserverSeverity
from observer.TopicsObserver import TopicsObserver, TopicStatus, TopicActions
from observer.SensorsObserver import SensorsObserver, SensorStatus
from observer.NodesObserver import NodesObserver, NodeStatus


class ObserverHandler(object):
    def __init__(self,
                 topics_cfg_file,
                 nodes_cfg_file,
                 sensors_cfg_file,
                 verbose=True,
                 use_startup_to=True,
                 ):

        # setup observers
        self.topics_obs = TopicsObserver(topics_cfg_file=topics_cfg_file, verbose=True, use_startup_to=use_startup_to)
        self.nodes_obs = NodesObserver(nodes_cfg_file=nodes_cfg_file, verbose=True, use_startup_to=use_startup_to)
        self.sensors_obs = SensorsObserver(sensors_cfg_file=sensors_cfg_file)

        # setup flags
        self.bVerbose = verbose
        self.bUseStartupTO = use_startup_to

        pass  # def __init__(...)

    def check_nodes(self):
        # modifies topic states
        if self.bVerbose:
            self.nodes_obs.print_status()

        nodes_status = self.nodes_obs.get_status()
        for node_name, node_status in nodes_status.items():
            if node_status == ObserverStatus.STARTING or node_status == ObserverStatus.UNOBSERVED:
                topics_names = self.topics_obs.has_node(node_name)
                for topic_name in topics_names:
                    if self.topics_obs.exists(topic_name):
                        self.topics_obs.observers[topic_name].stop_observation()

            if node_status == ObserverStatus.ERROR:
                topics_names = self.topics_obs.has_node(node_name)
                for topic_name in topics_names:
                    if self.topics_obs.exists(topic_name):
                        self.topics_obs.observers[topic_name].stop_observation()

                        # no need to observe and check topic, as node is erroneous
                        self.topics_obs.observers[topic_name].status = ObserverStatus.UNOBSERVED
                pass

            # start topic observer, once the node is operational
            if node_status == ObserverStatus.NOMINAL:
                topics_names = self.topics_obs.has_node(node_name)
                for topic_name in topics_names:
                    if self.topics_obs.exists(topic_name):
                        if self.topics_obs.observers[topic_name].status == ObserverStatus.UNOBSERVED:
                            self.topics_obs.observers[topic_name].start_observation()

        return nodes_status

    def check_topics(self):
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
                sensor_name = topic.sensor_name

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

    def start_observation(self):
        self.nodes_obs.start_observation()
        # topic observer will start automatically with node observer
        # self.topics_obs.start_observation()
        self.sensors_obs.start_observation()
        pass

    def stop_observation(self):
        self.nodes_obs.stop_observation()
        self.topics_obs.stop_observation()
        self.sensors_obs.stop_observation()
        pass


    def process(self):
        nodes_status = self.check_nodes()
        topics_status = self.check_topics()
        sensors_status = self.check_sensors()
        return nodes_status, topics_status, sensors_status

    def get_all_observers(self):
        return self.nodes_obs.get_observers(), self.topics_obs.get_observers(), self.sensors_obs.get_observers()

if __name__ == '__main__':

    rospy.init_node("Observer")
    # Go to class functions that do all the heavy lifting.
    try:
        obs = ObserverHandler('../../scripts/topics.ini', 'nodes.ini', 'sensors.ini', True)

        obs.start_observation()
        while not rospy.is_shutdown():
            status = obs.process()

            rospy.loginfo("Sleep...")
            time.sleep(1)
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()