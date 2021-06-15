#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import rospy, rosnode
import os
import time
import typing as typ

from observer.Observer import Observer, Observers, ObserverStatus, ObserverSeverity


def is_ros_node_running(node_name, verbose):
    # nodes = os.popen("rosnode list").readlines()
    nodes_list = rosnode.get_node_names()

    # debug info
    if verbose:
        rospy.logdebug("existing nodes: " + str(nodes_list))
        pass

    if node_name in nodes_list:
        return True
    else:
        return False


def kill_ros_node(node_name, verbose):
    if is_ros_node_running(node_name, verbose):
        if verbose:
            rospy.loginfo("-- rosnode kill " + node_name)
            pass

        os.system("rosnode kill " + node_name)
        # rosnode.kill_nodes([node_name])
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
            driver_name="",                 # type: str
            verbose=True,                   # type: bool
            ):

        # initialize super
        super(NodeObserver, self).__init__(str(node_name), entity_id, float(restart_timeout), verbose)

        # set node values
        self.num_restarts = 0
        self.max_restart_attempts = int(max_restart_attempts)
        self.restart_timeout = float(restart_timeout)
        self.driver_name = str(driver_name)
        self.t0 = -1
        self.t_running = -1

        # reset
        self.stop_observation()
        pass  # def __init__(...)

    def stop_observation(self):
        self.status = ObserverStatus.UNOBSERVED
        self.t0 = -1
        self.t_running = -1
        pass

    # important for high level logic to define when observation should start!
    def start_observation(self):
        self.status = ObserverStatus.STARTING
        self.t0 = rospy.get_rostime().now().to_sec()
        self.t_running = self.t0 + self.restart_timeout
        pass

    def update(self):
        if self.status == ObserverStatus.UNOBSERVED:
            if self.do_verbose():
                print("*  [" + self.name + "] not observing...")
                pass
            return
        else:
            # TODO(scm): this is the amaze logic, add state for failure which does the node restarting
            if self.is_running():
                self.status = ObserverStatus.NOMINAL
                if self.do_verbose():
                    print("*  [" + self.name + "] running")
                    pass
                return
            else:
                # TODO(scm) check here if restart attempts exceeded (etc)
                self.status = ObserverStatus.ERROR
                if self.do_verbose():
                    print("*  [" + self.name + "] not running")
                    pass
                return
            pass
        pass  # def update()

    def act(self):
        self._restart_node()
        pass  # def act()

    def _kill_node(self):
        return kill_ros_node(self.get_name(), self.do_verbose())

    def _restart_node(self):

        if self.num_restarts < self.max_restart_attempts:
            if self.do_verbose():
                rospy.loginfo("-- drivers node [" + self.name + "]")

            self.num_restarts += 1
            if self._kill_node():
                self.start_observation()
                return True
            else:
                return False
        elif self.num_restarts >= self.max_restart_attempts:
            self.num_restarts += 1
            rospy.logwarn("-- node [" + self.name + "] exceeded restart attempts!")

        return False

    def is_running(self):
        return is_ros_node_running(self.get_name(), self.do_verbose())

    # OLD LOGIC
    # def update(self):
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
    #             return ObserverStatus.ERROR
    #
    #     t_curr = rospy.get_rostime().now().to_sec()
    #     if self.t_running > t_curr:
    #         return ObserverStatus.STARTING  # all related TopicObserver need to drivers their observation! as long as the node is restarting
    #
    #     return ObserverStatus.NOMINAL
    #     pass  # def update()

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


class NodesObserver(Observers):

    def __init__(self,
                 cfg_file,                          # type: str
                 verbose=True,                      # type: bool
                 use_startup_to=True,               # type: bool
                 ):
        # call super constructor
        super(NodesObserver, self).__init__(
            cfg_file=cfg_file,
            verbose=verbose,
            name="NodesObserver"
        )

        # setup ID counter
        self.__cnt_id = 1   # always start with 1, 0 --> global

        for key, section in self.config_dict.items():
            # debugging
            if self.do_verbose():
                rospy.loginfo("Adding node %s" % str(key))
                pass

            # read configuration:
            self.observers[key] = NodeObserver(
                node_name=key,
                entity_id=str(section.get('entity_id', 'undefined')),
                max_restart_attempts=int(section.get('max_restart_attempts', '0')),
                restart_timeout=float(section.get('restart_timeout', '0.0')),
                verbose=verbose,
            )

            self.__cnt_id += 1
            pass  # for key, section in self.config_dict.items()

        pass  # def __init__(...)

    # def exists(self, node_name):
    #     return self.observers.has_key(node_name)
    #
    # def start_observation(self):
    #     for key, val in self.observers.items():
    #         val.start_observation()
    #
    # def stop_observation(self):
    #     for key, val in self.observers.items():
    #         val.stop_observation()
    #
    # def get_status(self):
    #     status = {}
    #     for key, val in self.observers.items():
    #         status[key] = val.get_status()
    #
    #     return status
    #
    # def get_observers(self):
    #     return self.observers
    #
    # def print_status(self):
    #     for name, status in self.get_status().items():
    #         print("- node:   [" + str(name) + "]:" + str(status.name))

    pass  # class NodesObservers(...)


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
