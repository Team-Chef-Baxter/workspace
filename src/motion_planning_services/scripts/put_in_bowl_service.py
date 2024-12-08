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
from motion_planning_services.srv import PutInBowl, PutInBowlResponse  # Replace with actual service names
from baxter_mover import get_mover

mover = None

def handle_put_in_bowl(req):
    global mover
    rospy.loginfo("Handling put in bowl by service.")
    mover.put_in_bowl()
    return PutInBowlResponse(success=True)

if __name__ == "__main__":
    rospy.init_node('put_in_bowl_service')
    while mover == None:
        mover = get_mover()
    rospy.loginfo("Got mover for put in bowl.")
    service = rospy.Service('/put_in_bowl_service', PutInBowl, handle_put_in_bowl)
    rospy.loginfo("PutInBowlService ready.")
    rospy.spin()
