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
from motion_planning_services.srv import HandToUser, HandToUserResponse  # Replace with actual service names
from baxter_mover import get_mover

mover = None

def handle_hand_to_user(req):
    global mover
    rospy.loginfo("Handling hand to user by service.")
    mover.hand_to_user()
    return HandToUserResponse(success=True)

if __name__ == "__main__":
    rospy.init_node('hand_to_user_service')
    service = rospy.Service('/hand_to_user_service', HandToUser, handle_hand_to_user)
    while mover == None:
        mover = get_mover()
    rospy.loginfo("Got mover for hand to user.")
    rospy.loginfo("HandToUserService ready.")
    rospy.spin()
