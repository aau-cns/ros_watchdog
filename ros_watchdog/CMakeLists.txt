cmake_minimum_required(VERSION 3.0.2)
project(ros_watchdog)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs autonomy_msgs)

add_service_files(
  FILES
  status.srv
#   Service2.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  autonomy_msgs
)

catkin_package(CATKIN_DEPENDS autonomy_msgs message_generation std_msgs )

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  scripts/Observer.py
  scripts/NodesObserver.py
  scripts/TopicsObserver.py
  scripts/Observer.py
  scripts/RosWatchdog.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
install(FILES
  README
  launch/default.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
