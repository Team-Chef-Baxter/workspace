cmake_minimum_required(VERSION 2.8.3)
project(motion_planning_services)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  baxter_tools
  geometry_msgs
  std_msgs
  object_detect
  baxter_shell
  message_generation
)

add_service_files(
  FILES
  PickUp.srv
  HandToUser.srv
  PutInBowl.srv
  Stir.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  std_msgs
  object_detect
  baxter_shell
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  message_runtime
)


#############
## Install ##
#############

install(PROGRAMS
   scripts/get_plan.py
   scripts/shell_planner.py
   scripts/master_planner.py
   scripts/motion_commands.py
   scripts/baxter_mover.py
   scripts/put_in_bowl_service.py
   scripts/pickup_service.py
   scripts/stir_service.py
   scripts/hand_to_user_service.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

