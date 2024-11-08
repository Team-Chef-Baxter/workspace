To run on the right pc:
from root of this (~/workspace):
./baxter.sh
source /opt/ros/noetic/setup.bash
source devel_isolated/setup.bash


to build (also from ~/workspace):
catkin_make_isolated -j2

For moving to arbitrary spot in space:
see baxter_shell/scripts/get_plan.py and move_baxter_arm(x, y, z)
