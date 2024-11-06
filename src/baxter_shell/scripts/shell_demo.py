#!/usr/bin/env python
import rospy
import argparse
import struct
import sys
import copy
import random
import rospy
import rospkg
from math import sqrt, cos, sin, pi

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)
import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
class Demo:
    def __init__(self):
        limb = 'right'
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        self.open = True
    
    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            # if self._verbose:
            #     print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
            #              (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            # if self._verbose:
            #     print("IK Joint Solution:\n{0}".format(limb_joints))
            #     print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints
    
    def move_baxter_arm(self, x=0.2, y=0.5, z=0.1):
        # rospy.init_node('move_baxter_arm')

        right_limb = baxter_interface.limb.Limb('right')

        current_angles = right_limb.joint_angles()

        target_position = {
            'right_s0': current_angles['right_s0'],
            'right_s1': current_angles['right_s1'],
            'right_e0': current_angles['right_e0'],
            'right_e1': current_angles['right_e1'],
            'right_w0': current_angles['right_w0'],
            'right_w1': current_angles['right_w1'],
            'right_w2': current_angles['right_w2']
        }

        

        # ik_position = {
        #     'position': {'x': x, 'y': y, 'z': z},
        #     'orientation': {'x': 0.0, 'y': 1.0, 'z': 0.0, 'w': 0.0}
        # }

        overhead_orientation = Quaternion(
                                x=0.0249590815779, # used to be -
                                y=0.999649402929,
                                z=0.00737916180073,
                                w=0.00486450832011)
        ik_request = (Pose(
                position=Point(x=x, y=y, z=z),
                orientation=overhead_orientation))

        # right_limb.set_joint_positions(ik_position)
        out_joints = self.ik_request(ik_request)
        if out_joints:
            right_limb.move_to_joint_positions(out_joints)
            rospy.loginfo(f"Moving right arm to x: {x}, y: {y}, z: {z}")
        else:
            rospy.loginfo("Not a valid end position")
        if self.open:
            self._gripper.close()
        else:
            self._gripper.open()
        rospy.sleep(1.0)
def main():
    rospy.init_node("shell_demo")
    demo_obj = Demo()

    while True:
        x_pos = float(input("Enter x_loc of where to move right arm"))
        y_pos = float(input("Enter y_loc of where to move right arm"))
        z_pos = float(input("Enter z_loc of where to move right arm"))
        demo_obj.move_baxter_arm(x_pos, y_pos, z_pos)



if __name__ == "__main__":
    main()