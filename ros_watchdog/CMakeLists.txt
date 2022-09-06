cmake_minimum_required(VERSION 3.0.2)
project(ros_watchdog)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs watchdog_msgs rospy)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS watchdog_msgs std_msgs )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  nodes/WatchdogNode.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  README
  launch/default.launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
