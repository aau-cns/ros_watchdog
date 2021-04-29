#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import rospy
import os
import time
from enum import Enum, unique
import typing as typ
import configparser

from observer.utils.Enums import OrderedEnum


@unique
class ObserverStatus(OrderedEnum):
    UNOBSERVED = 0         # not started yet
    NOMINAL = 1             # -> NOMINAL
    STARTING = 2            # -> NOMINAL
    NONCRITICAL = 4         # -> NON CRITICAL (message rate)
    ERROR = 8               # -> FAIL (depending on severity)
    pass  # class ObserverStatus


@unique
class ObserverSeverity(Enum):
    LOW = 0                 # -> NONCRITICAL errors
    MODERATE = 1            # -> INCONVENIENT errors
    HIGH = 2                # -> SEVERE FAILURES
    FATAL = 3               # -> ABORT
    UNDEFINED = -1          # not started yet
    pass  # class ObserverStatus


class Observer(object):
    """A general interface for any observer"""

    def __init__(self,
                 name,              # type: str
                 id,                # type: int
                 timeout=0.0,       # type: float
                 verbose=False,     # type: bool
                 ):
        # type: (...) -> None
        """Constructor for any observer"""

        # setup observer parameters
        self.name = name                            # type: str
        self.id = id                                # type: int
        self.timeout = timeout                      # type: float

        # set initial status to unobserved
        self.status = ObserverStatus.UNOBSERVED     # type: ObserverStatus
        self.severity = ObserverSeverity.UNDEFINED  # type: ObserverSeverity

        # setup flags
        self.__bVerbose = verbose                   # type: bool

        pass  # def __init__(...)

    ####################
    # PUBLIC METHODS
    ####################

    def start_observation(self):
        pass

    def stop_observation(self):
        pass

    def do_verbose(self):
        # type: (...) -> bool
        """Returns true if verbose is active"""
        return self.__bVerbose

    def get_name(self):
        return self.name

    def get_status(self):
        return self.status

    def get_severity(self):
        return self.severity

    def get_id(self):
        return self.id

    pass  # class Observer()


class Observers(object):

    def __init__(self,
                 cfg_file,
                 name="Observations",
                 verbose=False,
                 ):
        # preliminary checks for cfg file
        assert (os.path.exists(cfg_file))

        # setup observer parameters
        self.name = name                            # type: str
        self.cfg_file = cfg_file                    # type: str

        # set initial observers and statuses
        self.observers = {}                         # type: typ.Dict[str, Observer]
        self.statuses = {}                          # type: typ.Dict[str, ObserverStatus]
        self.status_changes = {}                    # type: typ.Dict[str, ObserverStatus]

        # setup flags
        self.__bVerbose = verbose                   # type: bool

        # read config
        self.config = self._read_config()           # type: configparser.ConfigParser
        pass

    ####################
    # PUBLIC METHODS
    ####################

    def start_observation(self):
        for obs in self.observers.values():
            obs.start_observation()
            pass
        pass

    def stop_observation(self):
        for obs in self.observers.values():
            obs.stop_observation()
            pass
        pass

    def exists(self, name):
        # type: (...) -> bool
        # return self.observers.has_key(name)
        # python 3 change
        return name in self.observers

    def update_statuses(self):
        # debug info
        if self.do_verbose():
            rospy.loginfo("%s == updating statuses" % self.get_name())
            pass

        # new variables
        new_statuses = {}
        new_changes = {}

        # check all statueses
        for key, val in self.observers.items():
            obs_status = val.get_status()
            new_statuses[key] = obs_status

            # check if status has changed
            if key in self.statuses.keys():
                # key present, check if value has changed
                if not self.statuses[key] == obs_status:
                    # status has changed, save change
                    new_changes[key] = obs_status

                    if self.do_verbose():
                        rospy.loginfo(
                            "%s == %s status changed: %d (%s)"
                            % (self.get_name(), str(key), obs_status.value,str(obs_status))
                        )
                        pass
                    pass
                pass
            else:
                # key not present, thus status change
                new_changes[key] = obs_status

                if self.do_verbose():
                    rospy.loginfo(
                        "%s == %s status added: %d (%s)"
                        % (self.get_name(), str(key), obs_status.value,str(obs_status))
                    )
                    pass
                pass
            pass

        # save statuses and changes
        self.statuses = new_statuses
        self.status_changes = new_changes

        # return current statuses
        return self.statuses

    ####################
    # PROTECTED METHODS
    ####################

    def _read_config(self):
        config = configparser.ConfigParser()
        config.sections()
        config.read(self.cfg_file)
        return config

    ####################
    # GETTER
    ####################

    def get_name(self):
        return self.name

    def get_cfg_file(self):
        return self.cfg_file

    def get_statuses(self):
        return self.statuses

    def get_status_changes(self):
        return self.status_changes

    def get_observers(self):
        return self.observers

    ####################
    # BOOLEANS
    ####################

    def do_verbose(self):
        # type: (...) -> bool
        """Returns true if verbose is active"""
        return self.__bVerbose

    pass  # class Observers()
