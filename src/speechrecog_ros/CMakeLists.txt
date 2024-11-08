cmake_minimum_required(VERSION 3.0.2)
project(speechrecog_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  SaladCommand.msg
  SaladRecipe.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/speech_recog.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
