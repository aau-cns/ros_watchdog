#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import typing as typ
from enum import Enum, unique

from observer.Observer import Observer, Observers, ObserverStatus
from observer.TopicsObserver import TopicsObserver
from observer.NodesObserver import NodesObserver


class Watchdog(object):

    @unique
    class States(Enum):
        STOPPED = 0
        INITIALIZING = 1
        STARTING = 2
        RUNNING = 3
        pass  # class Watchdog.States

    @ unique
    class ObserverKeys(Enum):
        TOPIC = 0
        NODE = 1
        DRIVER = 2
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
                # observer = NodesObserver(cfg_file, verbose=self.__bVerbose)
                pass
            elif ckey == Watchdog.ObserverKeys.DRIVER:
                # observer = DriversObserver(cfg_file, verbose=self.__bVerbose) TODO(scm)
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
              timeout_,                         # type: float
              ):
        # set starting state
        self.__state = Watchdog.States.STARTING
        rospy.logdebug("Starting Watchdog ...")

        # start observations
        for obs in self.__observers.values():
            obs.start_observation()
            pass

        rospy.logdebug("Starting Watchdog -- DONE")
        pass  # def start

    def watch(self):
        pass  # def watch

    def stop(self):
        pass  # def stop

    ####################
    # GETTER
    ####################

    def get_state(self):
        # type: (...) -> Watchdog.States
        """Returns the current state of the watchdog"""
        return self.__state

    def get_status_global(self):
        global_status = ObserverStatus.UNOBSERVED
        info_string = ""

        # check the max (=worst) status of each observation
        for key, obs in self.__observers.items():
            new_max = max(obs.get_statuses().values())

            # debug
            if self.__bVerbose:
                rospy.loginfo("Global Status %s: %d" % (str(key), new_max.value))
                pass

            global_status = max(global_status, new_max)
            info_string += "Status %s: %d\n" % (str(key), new_max.value)
            pass

        return global_status.value, info_string
    pass  # class Watchdog
