#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import rospy
import os
import time
from enum import Enum, unique
import configparser


@unique
class ObserverStatus(Enum):
    NOMINAL = 0             # -> NOMINAL
    STARTING = 1            # -> NOMINAL
    NONCRITICAL = 2         # -> NON CRITICAL (message rate)
    ERROR = 3               # -> FAIL (depending on severity)
    UNOBSERVED = -1         # not started yet
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
