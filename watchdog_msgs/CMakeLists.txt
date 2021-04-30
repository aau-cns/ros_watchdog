cmake_minimum_required(VERSION 3.0.2)
project(watchdog_msgs)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Action.msg
  ActionStamped.msg
  Status.msg
  StatusStamped.msg
  StatusChangesArray.msg
  StatusChangesArrayStamped.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  Start.srv
  Reset.srv
)

add_action_files(
  DIRECTORY action
  FILES HandleError.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)
