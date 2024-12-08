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
from motion_planning_services.srv import PickUp, PickUpResponse  # Replace with actual service names
from baxter_mover import get_mover
from master_planner import get_tag_locations

mover = None
tag_locations = None

def handle_pickup(req):
    global mover
    global tag_locations
    rospy.loginfo(f"Handling pick up for object {req.object_class} by service.")
                
    real_loc = mover.transform_pt([req.x, req.y, req.z])
    real_loc = real_loc[0]
    obj = req.object_class
    if obj == 'strawberry':
        z_offset = 0.018
    elif obj == 'orange':
        z_offset = 0.035
    elif obj == 'plum':
        z_offset = 0.022
    else:
        z_offset = 0.039
    offsets = [[0,0], [0.02, 0.01], [-0.02, -0.01], [0.02, -0.01], [-0.02, 0.01], [0, 0.01]]
    for i in range(len(offsets)):
        curr_o = offsets[i]
        mover.pickup(real_loc[0]+curr_o[0], real_loc[1]+curr_o[1], real_loc[2]+z_offset)
        rospy.loginfo("TRYING TO GET NEW LOC")
        
        tag_locations = get_tag_locations()
        new_loc = tag_locations.get(obj)
        diff = new_loc[2] - location[2]
        rospy.loginfo("Diff is %f", diff)
        # break
        if (diff > 0.005):
            rospy.loginfo("Got it")
            return True
        else:
            rospy.sleep(5)
            location = new_loc

    return PickUpResponse(success=False)


if __name__ == "__main__":
    rospy.init_node('pickup_service')
    pickup_service = rospy.Service('/pickup_service', PickUp, handle_pickup)
    while mover == None:
        mover = get_mover()
    rospy.loginfo("Got mover for pick up.")
    rospy.loginfo("PickUpService ready.")
    rospy.spin()
