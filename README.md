To run on the right pc:
from root of this (~/workspace):
bash
./baxter.sh


to build (also from ~/workspace):
catkin_make_isolated -j2

For moving to arbitrary spot in space:
see baxter_shell/scripts/get_plan.py and move_baxter_arm(x, y, z)

to run Chef Baxter:
- Human-robot interaction (speech and voice AI recognition):
  - rosrun speechrecog_ros speech_recog.py
- Object recognition with depth camera:
  - rosrun speechrecog_ros yolo_detect.py
- Motion planning:
  - roslaunch motion_planning_services moveit_init.launch
  - roslaunch motion_planning_services motion_planning_services.launch 
