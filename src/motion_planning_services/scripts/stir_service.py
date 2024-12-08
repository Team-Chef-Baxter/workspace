#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from sensor_msgs.msg import (
    JointState
)
import time
import baxter_interface
from scipy.spatial.transform import Rotation as R

from speechrecog_ros.msg import SaladRecipe, SaladCommand, DetectedObject
from std_msgs.msg import String
from motion_planning_services.srv import Stir, StirResponse  # Replace with actual service names
from baxter_mover import get_mover

mover = None

def handle_stir(req):
    global mover
    rospy.loginfo("Handling stir by service.")
    mover.stir()
    return StirResponse(success=True)

if __name__ == "__main__":
    rospy.init_node('stir_service')
    while mover == None:
        mover = get_mover()
    rospy.loginfo("Got mover for stir.")
    service = rospy.Service('/stir_service', Stir, handle_stir)
    rospy.loginfo("StirService ready.")
    rospy.spin()
