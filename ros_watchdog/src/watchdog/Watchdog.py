#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import typing as typ
from enum import unique

from observer.Observer import Observer, Observers, ObserverStatus, ObserverAction
from observer.TopicsObserver import TopicsObserver
from observer.NodesObserver import NodesObserver
from observer.DriversObserver import DriversObserver
from observer.utils.Enums import OrderedEnum


class Watchdog(object):

    @unique
    class States(OrderedEnum):
        STOPPED = 0
        INITIALIZING = 1
        STARTING = 2
        RUNNING = 3
        pass  # class Watchdog.States

    @ unique
    class ObserverKeys(OrderedEnum):
        GLOBAL = 0
        TOPIC = 1
        NODE = 2
        DRIVER = 3
        pass  # class Watchdog.ObserverKeys

    def __init__(self,
                 verbose=False,
                 ):

        # setup state machine
        self.__state = Watchdog.States.STOPPED      # type: Watchdog.States

        # setup observers
        self.__config_files = {}                    # type: typ.Dict[Watchdog.ObserverKeys, str]
        self.__observers = {}                       # type: typ.Dict[Watchdog.ObserverKeys, Observers]

        # setup flags
        self.__bVerbose = verbose                   # type: bool

        pass  # def __init__(...)

    def init(self,
             config_files_,                         # type: typ.Dict[Watchdog.ObserverKeys, str]
             ):
        # set initializing state
        self.__state = Watchdog.States.INITIALIZING
        rospy.logdebug("Initializing Watchdog ...")

        # reset observers
        self.__observers = {}
        self.__config_files = config_files_

        # add observer for each key
        for ckey in config_files_.keys():
            # check if key is valid
            if ckey not in Watchdog.ObserverKeys:
                rospy.logerr("Unknown key %s" % str(ckey))
                return

            # add observer to list of observers
            observer = None
            cfg_file = config_files_[ckey]
            if ckey == Watchdog.ObserverKeys.TOPIC:
                observer = TopicsObserver(cfg_file, verbose=self.__bVerbose)
                pass
            elif ckey == Watchdog.ObserverKeys.NODE:
                observer = NodesObserver(cfg_file, verbose=self.__bVerbose)
                pass
            elif ckey == Watchdog.ObserverKeys.DRIVER:
                observer = DriversObserver(cfg_file, verbose=self.__bVerbose)
                pass
            else:
                rospy.logerr("Key not implemented %s" % str(ckey))
                pass

            # save observer
            self.__observers[ckey] = observer
            pass  # for ckey in config_files_.keys()
        rospy.logdebug("Initializing Watchdog -- DONE")
        pass  # def init

    def start(self,
              timeout_,                             # type: float
              ):
        # set starting state
        self.__state = Watchdog.States.STARTING
        rospy.logdebug("Starting Watchdog ...")

        # start observations
        for obs in self.__observers.values():
            obs.start_observation()
            pass

        # set running state
        self.__state = Watchdog.States.RUNNING
        rospy.logdebug("Starting Watchdog -- DONE")
        pass  # def start

    def watch(self):
        # check if in correct state
        if self.__state == Watchdog.States.RUNNING:
            # update statuses
            for obs in self.__observers.values():
                obs.update_statuses()
                pass
            pass
        else:
            rospy.logwarn("Did not start watchdog yet, cannot watch. O.O")
        pass  # def watch

    def act(self,
            observer_key,
            entity_key,
            action_type,
            ):
        # (...) -> None
        """performs action for asset in any observer"""

        # check if action is to do nothing
        if action_type == ObserverAction.NOTHING:
            return

        # convert observer key and check if in range
        observer_key = Watchdog.ObserverKeys(observer_key)
        if Watchdog.ObserverKeys.DRIVER.GLOBAL < observer_key <= Watchdog.ObserverKeys.DRIVER:
            rospy.logerr("Observer type not implemented %s" % observer_key)
            return

        # Get observer of entity
        obs_list = self.__observers[observer_key].get_observers_with_id(entity_key)

        # check number of observers
        if len(obs_list) > 1:
            rospy.logwarn("Have multiple observers (type: %s) for entity %s\n\t"
                          "performing action for all of them" %
                          (observer_key, str(entity_key)))
            pass
        elif len(obs_list) == 0:
            rospy.logerr("Do not have any observers (type: %s) for entity %s\n"
                         "\tCannot perform any action" %
                         (observer_key, str(entity_key)))
            # TODO(scm): try to still perform action here by setting up new observer or similar
            pass

        # perform action for all observer entities
        for obs in obs_list:
            # in case of obs=topic, type=restart node/restart driver
            # we need knowledge of the node/driver
            # we need to stop observation for node/and driver

            # decide on action depending on obs key
            if observer_key == Watchdog.ObserverKeys.TOPIC:
                # get node and driver (if needed)
                node = self.__observers[Watchdog.ObserverKeys.NODE].get_observer_with_name(obs.node_name)
                driver = self.__observers[Watchdog.ObserverKeys.DRIVER].get_observer_with_name(obs.driver_name)

                # check if node exists
                if action_type >= ObserverAction.RESTART_NODE:
                    obs.stop_observation()
                    node.stop_observation()

                    # check for all actions to perform
                    if action_type == ObserverAction.RESTART_DRIVER:
                        # in case driver should be restarted
                        driver.stop_observation()
                        driver.act(action_type=action_type)
                        driver.start_observation()
                    else:
                        # otherwise perform node actions
                        node.act(action_type=action_type)
                        pass

                    # restart observations (= reset)
                    node.start_observation()
                    obs.start_observation()
                pass

            elif observer_key == Watchdog.ObserverKeys.NODE:
                driver = self.__observers[Watchdog.ObserverKeys.DRIVER].get_observer_with_name(obs.driver_name)

                # check if action should be performed
                if action_type >= ObserverAction.RESTART_NODE:
                    obs.stop_observation()

                    # check for all actions to perform
                    if action_type == ObserverAction.RESTART_DRIVER:
                        # in case driver should be restarted
                        driver.stop_observation()
                        driver.act(action_type=action_type)
                        driver.start_observation()
                    else:
                        # otherwise perform node actions
                        obs.act(action_type=action_type)
                        pass

                    # restart observations (= reset)
                    obs.start_observation()
                pass

            elif observer_key == Watchdog.ObserverKeys.DRIVER:
                # check if action to perform is correct (driver only has information about itself)
                if action_type == ObserverAction.RESTART_DRIVER:
                    # in case driver should be restarted
                    obs.stop_observation()
                    obs.act(action_type=action_type)
                    obs.start_observation()
                    pass
                pass
            else:
                # not implemented
                pass

            pass  # for obs in obs_list

        pass  # def act()

    def stop(self):
        pass  # def stop

    def __act_restart_node(self,
                           ):


        pass

    ####################
    # GETTER
    ####################

    def get_state(self):
        # type: (...) -> Watchdog.States
        """Returns the current state of the watchdog"""
        return self.__state

    def get_status_global(self,
                          as_int=True,
                          with_info=True,
                          with_id=False,
                          ):
        global_status = ObserverStatus.UNOBSERVED
        info_string = ""
        max_id = ""

        # check for current state of WD
        if self.__state == Watchdog.States.STOPPED:
            info_string = "Watchdog is not running"
            pass
        elif self.__state == Watchdog.States.STARTING or self.__state == Watchdog.States.INITIALIZING:
            global_status = ObserverStatus.STARTING
            info_string = "Watchdog is starting..."
            pass
        else:
            # check the max (=worst) status of each observation
            for key, obs in self.__observers.items():
                stats = obs.get_statuses()
                vals = stats.values()
                if len(vals) > 0:
                    # new_max = max(vals)
                    new_max_id = max(stats, key=stats.get)
                    new_max = stats[new_max_id]

                    # debug
                    if self.__bVerbose:
                        rospy.loginfo("Global Status %s: %d (%s)"
                                      % (str(key), new_max.value, str(new_max)))
                        pass

                    if new_max > global_status:
                        global_status = new_max
                        max_id = self.get_asset_id(key, new_max_id)
                        pass
                    # global_status = max(global_status, new_max)
                    info_string += "Status %s: %d\n" % (str(key), new_max.value)
                    pass
                else:
                    info_string += "Status %s: no entries to watch\n" % (str(key))

                    # debug
                    if self.__bVerbose:
                        rospy.loginfo("Global Status %s: no entries to watch"
                                      % (str(key)))
                        pass
                    pass
            pass

        if as_int:
            global_status = global_status.value
            pass

        if with_info:
            if with_id:
                return global_status, info_string, max_id
            else:
                return global_status, info_string
        else:
            return global_status
        pass

    def get_status_all(self):
        statuses = {}
        statuses[Watchdog.ObserverKeys.GLOBAL] = \
            {"": self.get_status_global(as_int=False, with_info=False)}

        # check for current state of WD
        if self.__state == Watchdog.States.RUNNING:
            for key, obs in self.__observers.items():
                statuses[key] = obs.get_statuses()
                pass
            pass

        return statuses
        pass  # get_status_all()

    def get_status_changes(self):
        changes = {}
        # check for current state of WD
        if self.__state == Watchdog.States.RUNNING:
            for key, obs in self.__observers.items():
                change = obs.get_status_changes()

                # check if change happend
                if len(change) > 0:
                    changes[key] = change
                    pass  # if len(change) > 0
                pass  # for key, obs in self.__observers.items()
            pass  # if self.__state == Watchdog.States.RUNNING

        return changes
        pass  # def get_status_changes()

    def get_asset_id(self,
                     observers_key,
                     asset_key,
                     ):
        if observers_key == Watchdog.ObserverKeys.GLOBAL:
            return "global"
        else:
            return self.__observers[observers_key].get_observer_id(key=asset_key)
        pass  # def get_asset_id(...)

    pass  # class Watchdog
