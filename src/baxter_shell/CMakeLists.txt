cmake_minimum_required(VERSION 2.8.3)
project(baxter_shell)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  baxter_tools
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PickUp.srv
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  baxter_tools
)


#############
## Install ##
#############

install(PROGRAMS
   scripts/shell_demo.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

