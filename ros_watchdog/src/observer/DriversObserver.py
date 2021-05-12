#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import rospy
import os
import subprocess
import typing as typ

from observer.Observer import Observer, Observers, ObserverStatus, ObserverAction


class DriverObserver(Observer):

    def __init__(
            self,
            name,                                   # type: str
            entity_id,                              # type: str
            timeout=0.0,                            # type: typ.Union[float, str]
            dirname='',                             # type: str
            check_script='',                        # type: str
            restart_script='',                      # type: str
            restart_attempts=1,                     # type: typ.Union[int, str]
            verbose=False,                          # type: bool
            ):

        # initialize super
        super(DriverObserver, self).__init__(name, entity_id, float(timeout), verbose)

        # setup parameters
        self.dirname = dirname
        self.check_script = check_script
        self.restart_script = restart_script
        self.restart_attempts = int(restart_attempts)

        # setup files
        if self.restart_script.startswith("/"):
            self.check_script_file = self.check_script
        else:
            self.check_script_file = os.path.join(self.dirname, self.check_script)
            pass
        if self.restart_script.startswith("/"):
            self.restart_script_file = self.restart_script
        else:
            self.restart_script_file = os.path.join(self.dirname, self.restart_script)
            pass

        # setup counters
        self.__cnt_restarts = 0
        self.clear_stats()
        pass

    ####################
    # OBSERVER METHODS
    ####################

    def start_observation(self):
        # type: (...) -> None

        # start observation and call
        self.status = ObserverStatus.STARTING

        pass  # def start_observation()

    def stop_observation(self):
        self.__cnt_restarts = 0
        self.status = ObserverStatus.UNOBSERVED
        pass  # def stop_observation()

    def act(self, **kwargs):
        # if action_type == ObserverAction.RESTART_DRIVER:
        #     # perform a drivers
        #     self._restart()
        #     pass
        # else:
        #     if self.do_verbose():
        #         rospy.logwarn("* [%s] action %s not implemented" %
        #                       (self.get_name(), str(action_type)))
        #         pass  # debug
        #     pass  # if action_type == ObserverAction.RESTART_DRIVER
        self._restart()
        pass  # def act(...)

    def update(self):
        # type: (...) -> ObserverStatus
        """Updates the current status of driver and returns it"""

        # debug
        if self.do_verbose():
            rospy.loginfo("* [%s] updating status - current %s" % (self.get_name(), self.status))
            pass

        # check if observation is running
        if self.status == ObserverStatus.UNOBSERVED:
            return self.status

        # check if driver is running
        # TODO(scm): perform systemctl checks here
        is_running = self._check_driver_systemctl()

        if is_running:
            self.status = ObserverStatus.NOMINAL
            if self.do_verbose():
                rospy.loginfo("* [%s] driver IS running" % self.get_name())
                rospy.loginfo("* - status: %s" % self.status)
                pass
            pass
        else:
            # if the driver is not running check what the current status is
            if self.status == ObserverStatus.STARTING and self.__cnt_restarts < self.restart_attempts:
                # perform drivers if we are in starting phase and have not reached max_restart attempts
                if self.do_verbose():
                    rospy.loginfo("* [%s] driver still starting" % self.get_name())
                    rospy.loginfo("* - status: %s" % self.status)
                    pass
                self.act()
                pass
            else:
                # enter error state
                self.status = ObserverStatus.ERROR
                if self.do_verbose():
                    rospy.loginfo("* [%s] driver NOT running" % self.get_name())
                    rospy.loginfo("* - status: %s" % self.status)
                    pass
                pass  # f self.status == ObserverStatus.STARTING and self.__cnt_restarts < self.restart_attempts
            pass  # (if)else is_running

        pass  # def get_status()

    ####################
    # PROTECTED METHODS
    ####################

    def _check_driver_systemctl(self):
        # TODO(scm): perform systemctl checks here
        if self.do_verbose():
            rospy.loginfo("* [%s] checking status - file %s" % (self.get_name(), str(self.check_script_file)))
            pass
        rc = subprocess.call(self.check_script_file)

        # return success code (0) or failure code (any other)
        if rc == 0:
            return True
        else:
            return False
        pass

    def _restart(self):
        # increase drivers counter
        self.__cnt_restarts += 1

        if self.do_verbose():
            rospy.logwarn("**** [%s] restarting sensor - attempt number %d" % (self.get_name(), self.__cnt_restarts))
            rospy.loginfo("**** -- executing %s" % str(self.restart_script_file))
            pass

        # execute restart script
        rc = subprocess.call(self.restart_script_file, shell=True)
        if rc == 0:
            print('success..')
            return True
        else:
            print('failed...')
            return False

        pass  # def _restart()

    def clear_stats(self):
        self.num_restarts = 0
        self.status = ObserverStatus.NOMINAL
        pass


class DriversObserver(Observers):

    def __init__(self,
                 cfg_file,                          # type: str
                 verbose=True,                      # type: bool
                 ):
        # call super constructor
        super(DriversObserver, self).__init__(
            cfg_file=cfg_file,
            verbose=verbose,
            name="DriversObserver"
        )

        # setup ID counter
        self.__cnt_id = 1   # always start with 1, 0 --> global

        self.dirname = os.path.dirname(os.path.abspath(cfg_file))

        for key, section in self.config_dict.items():
            # debugging
            if self.do_verbose():
                rospy.loginfo("Adding driver %s" % str(key))
                pass

            # read configuration:
            self.observers[key] = DriverObserver(
                name=key,
                entity_id=str(section.get('entity_id', 'undefined')),
                dirname=self.dirname,
                check_script=str(section.get('check_script', '')),
                restart_script=str(section.get('restart_script', '')),
                restart_attempts=int(section.get('restart_attempts', '0')),
                verbose=verbose
            )

            self.__cnt_id += 1

            pass  # for key, section in self.config_dict.items()

        pass  # def __init__(...)

    pass  # class DriversObserver(...)
