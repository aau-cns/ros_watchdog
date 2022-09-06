cmake_minimum_required(VERSION 3.0.2)
project(watchdog_bringup)
find_package(catkin REQUIRED COMPONENTS ros_watchdog)
catkin_package(CATKIN_DEPENDS ros_watchdog)
