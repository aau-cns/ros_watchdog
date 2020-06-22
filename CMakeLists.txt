cmake_minimum_required(VERSION 3.0.2)
project(ros_watchdog)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs autonomy_msgs)



catkin_package(CATKIN_DEPENDS message_runtime std_msgs autonomy_msgs)

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  scripts/Sensors.py
  scripts/TopicsObserver.py  
  scripts/ros_watchdog.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
install(FILES
  README
  launch/default.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
