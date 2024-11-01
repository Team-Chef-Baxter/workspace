search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=baxter.srdf
robot_name_in_srdf=baxter
moveit_config_pkg=baxter_moveit_config
robot_name=baxter
planning_group_name=left_arm
ikfast_plugin_pkg=baxter_left_arm_ikfast_plugin
base_link_name=left_arm_mount
eef_link_name=left_gripper_base
ikfast_output_path=/home/bhl2/workspace/src/baxter_left_arm_ikfast_plugin/src/baxter_left_arm_ikfast_solver.cpp
eef_direction="0 0 1"

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  --eef_direction $eef_direction\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
