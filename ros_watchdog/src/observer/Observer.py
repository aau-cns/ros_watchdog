#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import rospy
import os
import time
from enum import Enum, unique
import typing as typ
import configparser
import yaml

from observer.utils.Enums import OrderedEnum


@unique
class ObserverStatus(OrderedEnum):
    UNOBSERVED = 0         # not started yet
    NOMINAL = 1             # -> NOMINAL
    STARTING = 2            # -> NOMINAL
    NONCRITICAL = 4         # -> NON CRITICAL (message rate)
    ERROR = 8               # -> FAIL (depending on severity)
    pass  # enum ObserverStatus


@unique
class ObserverAction(OrderedEnum):
    NOTHING = 0
    RESTART_NODE = 1
    RESTART_DRIVER = 2
    KILL_NODE = 4
    pass  # enum ObserverAction


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
                 name,                              # type: str
                 entity_id,                         # type: str
                 timeout=0.0,                       # type: float
                 verbose=False,                     # type: bool
                 ):
        # type: (...) -> None
        """Constructor for any observer"""

        # setup observer parameters
        self.name = name                            # type: str
        self.entity_id = entity_id                  # type: str
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
        self.status = ObserverStatus.STARTING
        pass

    def stop_observation(self):
        self.status = ObserverStatus.UNOBSERVED
        pass

    def update(self):
        """Updates the status of the observer"""
        pass

    def act(self,
            action_type=ObserverAction.NOTHING,     # type: ObserverAction
            ):
        pass

    ####################
    # GETTER
    ####################

    def get_name(self):
        # type: (...) -> str
        return self.name

    def get_status(self,
                   do_update=False,
                   ):
        # type: (...) -> ObserverStatus
        """returns the status of the observer"""
        # perform update if requested
        if do_update:
            self.update()
            pass
        return self.status

    def get_severity(self):
        # type: (...) -> ObserverSeverity
        return self.severity

    def get_id(self):
        # type: (...) -> str
        return self.entity_id

    ####################
    # BOOLEANS
    ####################

    def do_verbose(self):
        # type: (...) -> bool
        """Returns true if verbose is active"""
        return self.__bVerbose

    pass  # class Observer()


class Observers(object):

    def __init__(self,
                 cfg_file,                          # type: str
                 name="Observations",               # type: str
                 verbose=False,                     # type: bool
                 ):
        # setup observer parameters
        self.name = name                            # type: str
        self.cfg_file = cfg_file                    # type: str
        rospy.logdebug("Setting up %s" % (self.get_name()))

        if verbose:
            rospy.loginfo("%s == config file: %s" % (self.get_name(), self.cfg_file))

        # preliminary checks for cfg file
        assert (os.path.exists(cfg_file))

        # set initial observers and statuses
        self.observers = {}                         # type: typ.Dict[str, Observer]
        self.statuses = {}                          # type: typ.Dict[str, ObserverStatus]
        self.status_changes = {}                    # type: typ.Dict[str, ObserverStatus]

        # setup flags
        self.__bVerbose = verbose                   # type: bool

        # read config
        self.config = self._read_config()           # type: configparser.ConfigParser
        self.config_dict = self._get_config_dict()  # type: dict
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

        # check all statuses
        for key, val in self.observers.items():
            obs_status = val.get_status(do_update=True)
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

        if self.do_verbose():
            rospy.loginfo(
                "%s == updated statuses (length: %d)"
                % (self.get_name(), len(self.statuses))
            )

        # return current statuses
        return self.statuses

    ####################
    # PROTECTED METHODS
    ####################

    def _read_config(self):
        if self.cfg_file.endswith('.ini'):
            rospy.logwarn_once(
                "LEGACY WARNING: .ini file support will be removed in future releases."
            )
            config = configparser.ConfigParser()
            config.sections()
            config.read(self.cfg_file)
            pass
        elif self.cfg_file.endswith('.yaml') or self.cfg_file.endswith('.yml'):
            config = yaml.load(open(self.cfg_file), Loader=yaml.SafeLoader)
            pass
        else:
            rospy.logerr(
                "Unknown file type for config file %s"
                % (self.cfg_file)
            )
        return config

    ####################
    # I/O METHODS
    ####################

    def print_statuses(self):
        for name, status in self.statuses.items():
            rospy.loginfo("%s == asset [%s]: %s" %
                          (self.get_name(), str(name), str(status)))
            pass
        pass  # def print_statuses()

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

    def get_observer_with_name(self, name):
        # type: (...) -> typ.Optional[Observer]
        """returns the observer with the key name"""
        if self.exists(name):
            return self.observers[name]
        else:
            return None
        pass  # def get_observer_with_id(...)

    def get_observers_with_id(self, entity_id):
        # type: (...) -> typ.List[Observer]
        obs_list = []
        for key, obs in self.observers.items():
            if obs.get_id() == entity_id:
                obs_list.append(obs)
                pass  # if obs.get_id() == entity_id
            pass  # for key, obs in self.observers.items()

        if self.do_verbose():
            if len(obs_list) == 0:
                rospy.logwarn("%s == entity %s not found in observers" %
                              (self.get_name(), str(entity_id)))
                pass
            pass  # if self.do_verbose()

        return obs_list
        pass  # def get_observer_with_id(...)

    def get_observers(self):
        return self.observers

    def get_observer_id(self, key=None):
        if key is None:
            # TODO(scm)
            pass
        else:
            return self.observers[key].get_id()
        pass  # def get_observer_id(...)

    def _get_config_dict(self):
        # check if config is none (required in .yaml case)
        if self.config is None:
            # no elements in config
            rospy.logwarn("%s == ERROR: no assets in %s" % (self.get_name(), self.cfg_file))
            return {}

        # make sure config_dict is a dict
        config_dict = dict(self.config.items())

        # remove 'DEFAULT' key from dict (in .ini case DEFAULT is always present)
        config_dict.pop('DEFAULT', None)

        # check length of config items (required in .ini case)
        if len(config_dict) < 1:
            # no elements in config
            rospy.logwarn("%s == ERROR: no assets in %s" % (self.get_name(), self.cfg_file))
            return {}
        else:
            return config_dict
        pass  # _get_config_dict()

    ####################
    # BOOLEANS
    ####################

    def do_verbose(self):
        # type: (...) -> bool
        """Returns true if verbose is active"""
        return self.__bVerbose

    pass  # class Observers()
