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
from baxter_shell.srv import PickUp, PickUpRequest  # Replace with actual service names

overhead_orientation = Quaternion(
                                x=0.0249590815779, # used to be -
                                y=0.999649402929,
                                z=0.00737916180073,
                                w=0.00486450832011)

scale = 1.0
    # x: 0.7946775662824711
    # y: 0.041587616153320565
    # z: 0.10572616670845197
    #     x: 0.7024582811966628
    # y: 0.09185341909854253
    # z: 0.060440780624607615
center_bowl = [0.7724582811966628, 0.09185341909854253, 0.100440780624607615]
radius_bowl = 0.7794097889384775- 0.6879418901186068

mover = None

class BaxterMover:

    def __init__(self):
        # rospy.init_node('baxter_mover', anonymous=True)
        # self._limb = baxter_interface.Limb(limb)
        self._right_limb = baxter_interface.Limb('right')
        self._left_limb = baxter_interface.Limb('left')
        self.fix_time()
        self.init_moveit()
        self.neutral()
        self.make_cal()
        # self.stir()

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def transform_pt(self, point):
        # point is [x, y, z] from camera
        point = np.array(point)
        trans_pt = (self.r_mat @ point.T).T + self.t_vec
        trans_pt[0][0] += 0.03
        # trans_pt[0][2] += 0.01
        return trans_pt
    
    def make_cal(self):
        orange_pic_locs = np.array([
            [0.7884495231184032, 0.0747976915196221, 0.060294470887498994],
            [0.752279042654431, -0.1484103239533882, 0.05705757136226672],
            [0.7245566176068525, -0.14825844895877952, 0.07085353112918079]
            # [0.7291670226219468, -0.013633926338206507, 0.3093425146615328]
        ])
        # x: 0.7946449680716132    y: -0.1626693834846344    z: 0.052709847382641103
        # x: 0.7724988569142711    y: 0.003656492845555418   z: 0.05705667424658242
        # x: 0.8783673585018751    y: -0.1826688489824065    z: 0.055498466446806005
        # x: 0.825827980125295     y: -0.06133484428877023   z: 0.28534301063412115
        orange_pic_locs = np.array([
            [0.7946449680716132, -0.1626693834846344, 0.052709847382641103],
            [0.7724988569142711, 0.003656492845555418, 0.05705667424658242],
            [0.8783673585018751, -0.1826688489824065, 0.055498466446806005],
            [0.825827980125295, -0.06133484428877023, 0.28534301063412115]
        ])
        right_arm_locs = np.array([
            [-0.07869296, 0.17185737, 0.66],
            [0.13634792, 0.1732389, 0.594],
            [0.14614189, 0.1373064, 0.524]
            # [0.07880044, 0.14919694, 0.754]
        ])
        # x: 0.14772856   y: 0.17164546  z: 0.639
        # x: 0.0070819124 y: 0.16117646  z: 0.572
        # x: 0.20298031   y: 0.17754087  z: 0.695
        # x: 0.080802254  y: -0.03596205 z: 0.668
        right_arm_locs = np.array([
            [0.14772856, 0.17164546, 0.639],
            [0.0070819124, 0.16117646, 0.572],
            [0.20298031, 0.17754087, 0.695],
            [0.080802254, -0.03596205, 0.668]
        ])
        # Compute the centroids
        centroid_arm = np.mean(right_arm_locs, axis=0)
        centroid_cam = np.mean(orange_pic_locs, axis=0)

        # Center the points
        arm_centered = right_arm_locs - centroid_arm
        cam_centered = orange_pic_locs - centroid_cam

        # Compute the cross-covariance matrix
        H = arm_centered.T @ cam_centered

        # Singular Value Decomposition (SVD)
        U, S, Vt = np.linalg.svd(H)
        R_matrix = Vt.T @ U.T

        # Ensure a proper rotation (det(R) = 1)
        if np.linalg.det(R_matrix) < 0:
            Vt[-1, :] *= -1
            R_matrix = Vt.T @ U.T

        # Compute the translation vector
        t_vector = centroid_cam - R_matrix @ centroid_arm

        # Print the results
        print("Rotation matrix R:")
        print(R_matrix)

        print("\nTranslation vector t:")
        print(t_vector)

        # Validate the transformation
        transformed_points = (R_matrix @ right_arm_locs.T).T + t_vector
        print("\nTransformed points:")
        print(transformed_points)

        # Compare with orange_pic_locs
        print("\nDifference:")
        print(orange_pic_locs - transformed_points)

        test_pt = np.array([0.17376125,0.14817673,0.768])
        test_pt = np.array([-0.12853557, 0.13025814, 0.581])
        test_pt = np.array([0.18185805, 0.1578463, 0.499])
        test_pt = np.array([0.040170025, 0.16278283, 0.588])
        trans_pt = (R_matrix @ test_pt.T).T + t_vector
        self.r_mat = R_matrix
        self.t_vec = t_vector
    def current_right_state(self):
        """
        C{pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}}

          - 'position': Cartesian coordinates x,y,z in
                        namedtuple L{Limb.Point}
          - 'orientation': quaternion x,y,z,w in named tuple
                           L{Limb.Quaternion}
        """
        state_map = self._right_limb.endpoint_pose()
        return state_map['position']
    def current_left_state(self):
        """
        C{pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}}

          - 'position': Cartesian coordinates x,y,z in
                        namedtuple L{Limb.Point}
          - 'orientation': quaternion x,y,z,w in named tuple
                           L{Limb.Quaternion}
        """
        state_map = self._left_limb.endpoint_pose()
        return state_map['position']
    def neutral(self):
        # RIGHT NEUTRAL: x: 0.794370406419535 y: -0.14616875018957254 z: 0.26769252675751654
        pose_goal = Pose(
                position=Point(x=0.8, y=-0.15, z=0.26),
                orientation=overhead_orientation)
        self.right_group.set_pose_target(pose_goal)
        self.right_group.go(wait=True)

        # LEFT NEUTRAL:   position: x: 0.8635879504162788 y: 0.40325877049116904 z: 0.2772426634141517
        pose_goal = Pose(
                position=Point(x=0.8, y=0.4, z=0.26),
                orientation=overhead_orientation)
        self.left_group.set_pose_target(pose_goal)
        self.left_group.go(wait=True)

        time.sleep(2)

    def init_moveit(self):
        # sets up all the moveit stuff
        # initialize moveit_commander and rospy.

        #TimeFixer()
        joint_state_topic = ['joint_states:=/joint_states']
        
        # rospy.spin()
        # roscpp_initialize(sys.argv)
        moveit_commander.roscpp_initialize(joint_state_topic)
        # moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a RobotCommander object.  This object is
        # an interface to the robot as a whole.
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        # self.init_scene()
        self.right_group = moveit_commander.MoveGroupCommander("right_arm")
        self.left_group = moveit_commander.MoveGroupCommander("left_arm")
    
    def go_to(self, x, y, z, x_o, y_o, z_o, w):
        # Pose(
        #         position=Point(x=center_bowl[0]+0.04, y=center_bowl[1], z=center_bowl[2]+0.25),
        #         orientation=overhead_orientation)
        # Quaternion(   x=0.0249590815779, # used to be -
        #                         y=0.999649402929,
        #                         z=0.00737916180073,
        #                         w=0.00486450832011)
        pos = Point(x=x-0.1, y=y+0.05, z=z+0.25)
        ori = Quaternion(x=x_o, y=y_o, z=z_o, w=w)
        pose = Pose(position=pos, orientation=ori)
        pose_goal = pose
        self.left_group.set_pose_target(pose_goal)
        self.left_group.go(wait=True)
        self.left_group.stop()
        time.sleep(1)
    def stir(self):

        # # STIR COORD 1
        #   position: 
        #     x: 0.8385698299532868
        #     y: 0.034538432765428104
        #     z: -0.05999697772176437
        # orientation: 
        #     x: 0.1817816551756012
        #     y: 0.9384344673580036
        #     z: -0.10476810963060784
        #     w: 0.2744445727657004
        # # STIR COORD 2
        #   position: 
        #     x: 0.7562831345184086
        #     y: 0.15409830862355017
        #     z: -0.0726335942888011
        # orientation: 
        #     x: 0.1138372201050532
        #     y: 0.9472988775622068
        #     z: -0.11139316801370487
        #     w: 0.27795230887331646

        # # STIR COORD 3
        # # pose: 
        #   position: 
        #     x: 0.8839297644868203
        #     y: 0.20385850759656382
        #     z: -0.06885764268376211
        # orientation: 
        #     x: 0.07242461460861231
        #     y: 0.9520865491485894
        #     z: -0.08609293981352177
        #     w: 0.28438333960235784

        # # STIR COORD 4
        #   position: 
        #     x: 0.9338994597536608
        #     y: 0.10126504544666193
        #     z: -0.07209344821155442
        # orientation: 
        #     x: 0.11296713110635614
        #     y: 0.9479179133187138
        #     z: -0.1055518626932763
        #     w: 0.2784759615856897
        STIR1 = [0.8385698299532868, 0.034538432765428104, -0.05999697772176437,
            0.1817816551756012, 0.9384344673580036, -0.10476810963060784, 0.2744445727657004]

        STIR2 = [0.7562831345184086, 0.15409830862355017, -0.0726335942888011,
                0.1138372201050532, 0.9472988775622068, -0.11139316801370487, 0.27795230887331646]

        STIR3 = [0.8839297644868203, 0.20385850759656382, -0.06885764268376211,
                0.07242461460861231, 0.9520865491485894, -0.08609293981352177, 0.28438333960235784]

        STIR4 = [0.9338994597536608, 0.10126504544666193, -0.07209344821155442,
                0.11296713110635614, 0.9479179133187138, -0.1055518626932763, 0.2784759615856897]

            # move right arm out of way
        pose_goal = Pose(
                position=Point(x=0.8, y=-0.4, z=0.26),
                orientation=overhead_orientation)
        self.right_group.set_pose_target(pose_goal)
        self.right_group.go(wait=True)
        time.sleep(1)
        # self.go_to()
        # velocity control on left arm
        # joint_names = self._left_limb.joint_names()
        # zero_dict = dict([(joint, 0)
        #                  for i, joint in enumerate(joint_names)])
        # zero_dict["left_w2"] = 0.05
        # self._left_limb.set_joint_velocities(zero_dict)

        # move left above 
        tolerance = 0.01
        left_pos  = self.current_left_state()
        diff_x = center_bowl[0]+0.04 - left_pos[0]
        diff_y = center_bowl[1] - left_pos[1]
        diff_z = center_bowl[2]+0.25 - left_pos[2]
        if not (abs(diff_x) < tolerance and abs(diff_y) < tolerance and abs(diff_z) < tolerance):
            pose_goal = Pose(
                position=Point(x=center_bowl[0]+0.04, y=center_bowl[1], z=center_bowl[2]+0.25),
                orientation=overhead_orientation)
            self.left_group.set_pose_target(pose_goal)
            self.left_group.go(wait=True)
            self.left_group.stop()
            time.sleep(1)
        
        self.go_to(0.8385698299532868, 0.034538432765428104, -0.05999697772176437,
            0.1817816551756012, 0.9384344673580036, -0.10476810963060784, 0.2744445727657004)
        
        self.go_to(0.7562831345184086, 0.15409830862355017, -0.0726335942888011,
                0.1138372201050532, 0.9472988775622068, -0.11139316801370487, 0.27795230887331646)
        
        self.go_to(0.8839297644868203, 0.20385850759656382, -0.06885764268376211,
                0.07242461460861231, 0.9520865491485894, -0.08609293981352177, 0.28438333960235784)
        
        self.go_to(0.9338994597536608, 0.10126504544666193, -0.07209344821155442,
                0.11296713110635614, 0.9479179133187138, -0.1055518626932763, 0.2784759615856897)
        # # lower into bowl
        # # pose_goal = Pose(
        # #         position=Point(x=center_bowl[0]+0.04, y=center_bowl[1], z=center_bowl[2]+0.25),
        # #         orientation=overhead_orientation)
        # # self.left_group.set_pose_target(pose_goal)
        # # self.left_group.go(wait=True)
        # # self.left_group.stop()
        # time.sleep(2)
        # for i in range(3):
        #     # go up and down
        #     dir = (i+1) % 2
        #     if (dir == 0):
        #         dir = -1
        #     else:
        #         dir = 2
        #     x_offset = dir*(radius_bowl/2 - 0.015) # + 0.02
        #     pose_goal = Pose(
        #         position=Point(x=center_bowl[0]+x_offset, y=center_bowl[1], z=center_bowl[2]+0.25),
        #         orientation=overhead_orientation)
        #     self.left_group.set_pose_target(pose_goal)
        #     self.left_group.go(wait=True)
        #     self.left_group.stop()
        #     time.sleep(0.5)
        #     pose_goal = Pose(
        #         position=Point(x=center_bowl[0]+x_offset, y=center_bowl[1], z=center_bowl[2]+0.115),
        #         orientation=overhead_orientation)
        #     self.left_group.set_pose_target(pose_goal)
        #     worked = self.left_group.go(wait=True)
        #     self.left_group.stop()
        #     time.sleep(0.5)
        #     pose_goal = Pose(
        #         position=Point(x=center_bowl[0]+x_offset, y=center_bowl[1], z=center_bowl[2]+0.25),
        #         orientation=overhead_orientation)
        #     self.left_group.set_pose_target(pose_goal)
        #     self.left_group.go(wait=True)
        #     self.left_group.stop()
        #     time.sleep(0.5)
        
        # lift out of bowl
        pose_goal = Pose(
                position=Point(x=center_bowl[0]+0.04, y=center_bowl[1], z=center_bowl[2]+0.25),
                orientation=overhead_orientation)
        self.left_group.set_pose_target(pose_goal)
        self.left_group.go(wait=True)
        

    def put_in_bowl(self):
        # assumes object on magnet
        curr = self.current_right_state()

        # move left arm away
        pose_goal = Pose(
                position=Point(x=0.8, y=0.4, z=0.26),
                orientation=overhead_orientation)
        self.left_group.set_pose_target(pose_goal)
        self.left_group.go(wait=True)
        time.sleep(1)
        # move to right z
        # pose_goal = Pose(
        #         position=Point(x=curr[0], y=curr[1], z=center_bowl[2]+0.2),
        #         orientation=overhead_orientation)
        # self.right_group.set_pose_target(pose_goal)
        # self.right_group.go(wait=True)
        # time.sleep(2)
        # move to right x
        right_pos = self.current_right_state()
        tolerance = 0.01
        diff_x = center_bowl[0] - right_pos[0]
        diff_y = curr[1] - right_pos[1]
        diff_z = center_bowl[2]+0.2 - right_pos[2]
        if not (abs(diff_x) < tolerance and abs(diff_y) < tolerance and abs(diff_z) < tolerance):
            pose_goal = Pose(
                    position=Point(x=center_bowl[0], y=curr[1], z=center_bowl[2]+0.2),
                    orientation=overhead_orientation)
            self.right_group.set_pose_target(pose_goal)
            self.right_group.go(wait=True)
            time.sleep(2)

        # move to right y
        right_pos = self.current_right_state()
        tolerance = 0.01
        diff_x = center_bowl[0] - right_pos[0]
        diff_y = center_bowl[1] - right_pos[1]
        diff_z = center_bowl[2]+0.2 - right_pos[2]
        if not (abs(diff_x) < tolerance and abs(diff_y) < tolerance and abs(diff_z) < tolerance):
            pose_goal = Pose(
                    position=Point(x=center_bowl[0], y=center_bowl[1], z=center_bowl[2]+0.2),
                    orientation=overhead_orientation)
            self.right_group.set_pose_target(pose_goal)
            self.right_group.go(wait=True)

        # lower into bowl
        pose_goal = Pose(
                position=Point(x=center_bowl[0], y=center_bowl[1], z=center_bowl[2]+0.1),
                orientation=overhead_orientation)
        self.right_group.set_pose_target(pose_goal)
        self.right_group.go(wait=True)

        # now move away to drop it in
        pose_goal = Pose(
                position=Point(x=center_bowl[0], y=center_bowl[1]-0.2, z=center_bowl[2]+0.1),
                orientation=overhead_orientation)
        self.right_group.set_pose_target(pose_goal)
        self.right_group.go(wait=True)
    def hand_to_user(self):
        # move left away
        pose_goal = Pose(
                position=Point(x=0.8, y=0.4, z=0.26),
                orientation=overhead_orientation)
        self.left_group.set_pose_target(pose_goal)
        self.left_group.go(wait=True)
        time.sleep(1)

        # move to right x
        pose_goal = Pose(
                position=Point(x=0.8, y=0, z=0.26),
                orientation=overhead_orientation)
        self.right_group.set_pose_target(pose_goal)
        self.right_group.go(wait=True)
        time.sleep(1)
    def remove(self, angle):
        # ANGLE IN RADIANS, FROM BAXTER POV
        # determine end location (bowl rim)
        center_x = center_bowl[0]
        center_y = center_bowl[1]
        r = radius_bowl

        final_x = (r * np.cos(angle)) + center_x
        final_y = (r * np.sin(angle)) + center_y

        curr = self.current_right_state()

        # move to right x first
        big_r = r + 1
        inter_x = (big_r * np.cos(angle)) + center_x
        pose_goal = Pose(
                position=Point(x=inter_x, y=curr[1], z=center_bowl[2]+0.102),
                orientation=overhead_orientation)
        self.right_group.set_pose_target(pose_goal)
        self.right_group.go(wait=True) 
        time.sleep(1)
        inter_y = (big_r * np.sin(angle)) + center_x
        pose_goal = Pose(
                position=Point(x=inter_x, y=inter_y, z=center_bowl[2]+0.102),
                orientation=overhead_orientation)
        self.right_group.set_pose_target(pose_goal)
        self.right_group.go(wait=True) 

        time.sleep(1)
        pose_goal = Pose(
                position=Point(x=final_x, y=final_y, z=center_bowl[2]+0.102),
                orientation=overhead_orientation)
        self.right_group.set_pose_target(pose_goal)
        self.right_group.go(wait=True) 
        time.sleep(1)
        pass

    def shove_right(self):
        
        pass
    def pickup(self, x, y, z):
        rospy.loginfo("Moving to %f, %f, %f", x, y, z)
        z = z + 0.1
        # go to above the object
        pose_goal = Pose(
                position=Point(x=x, y=y, z=z+0.1),
                orientation=overhead_orientation)
        self.right_group.set_pose_target(pose_goal)
        self.right_group.go(wait=True)

        time.sleep(1.5)

        # plan to linearly descend
        # waypoints = []

        # wpose = self.right_group.get_current_pose().pose
        # wpose.position.z -= scale * 0.1  # Z steps
        # waypoints.append(copy.deepcopy(wpose))


        # # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # # which is why we will specify 0.01 as the eef_step in Cartesian
        # # translation.  We will disable the jump threshold by setting it to 0.0,
        # # ignoring the check for infeasible jumps in joint space, which is sufficient
        # # for this tutorial.
        # (plan, fraction) = self.right_group.compute_cartesian_path(
        #     waypoints, 0.01  # waypoints to follow  # eef_step
        # )
        # self.right_group.execute(plan, wait=True)
        pose_goal = Pose(
                position=Point(x=x, y=y, z=z),
                orientation=overhead_orientation)
        self.right_group.set_pose_target(pose_goal)
        self.right_group.go(wait=True)

        time.sleep(1.5)
        

        # plan to linearly ascend
        pose_goal = Pose(
                position=Point(x=x, y=y, z=z+0.1),
                orientation=overhead_orientation)
        self.right_group.set_pose_target(pose_goal)
        self.right_group.go(wait=True)
        time.sleep(1.5)

        return

    def init_scene(self):
        # box_pose = geometry_msgs.msg.PoseStamped()
        # box_pose.header.frame_id = "panda_hand"
        # box_pose.pose.orientation.w = 1.0
        # box_pose.pose.position.z = 0.11  # above the panda_hand frame
        # box_name = "box"
        # scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = -0.5
        box_pose.pose.position.x = 0.66
        box_name = "table"
        self.scene.add_box(box_name, box_pose, size=(0.8, 1.21, 0.66))

        left_wall_pose = geometry_msgs.msg.PoseStamped()
        left_wall_pose.header.frame_id = "base"
        left_wall_pose.pose.orientation.w = 1.0
        left_wall_pose.pose.position.x = 0
        left_wall_pose.pose.position.y = 0.6
        self.scene.add_box("left wall", left_wall_pose, size=(2, 0.1, 2))

        right_wall_pose = geometry_msgs.msg.PoseStamped()
        right_wall_pose.header.frame_id = "base"
        right_wall_pose.pose.orientation.w = 1.0
        right_wall_pose.pose.position.x = 0
        right_wall_pose.pose.position.y = -0.6
        self.scene.add_box("right wall", right_wall_pose, size=(2, 0.1, 2))

        back_wall_pose = geometry_msgs.msg.PoseStamped()
        back_wall_pose.header.frame_id = "base"
        back_wall_pose.pose.orientation.w = 1.0
        back_wall_pose.pose.position.x = 1.06
        back_wall_pose.pose.position.y = 0
        self.scene.add_box("back wall", back_wall_pose, size=(0.1, 1.21, 2))

        pass
    def joint_state_callback(self, msg):
        """
        Callback function for the JointState subscriber. Updates the header stamp to the current time
        and republishes the message.
        """
        # Update the header timestamp to the current time
        msg.header.stamp = rospy.Time.now()
        
        # Publish the modified message
        self.pub.publish(msg)

    def fix_time(self):
        # Create a publisher to the new topic
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Subscribe to the original joint_states topic
        rospy.Subscriber('/robot/joint_states', JointState, self.joint_state_callback)
        
    def demo(self):
        #     x: 0.7492572906843715 y: -0.19644058959803903 z: 0.042689133441078914
        _pickup_x = 0.7492572906843715
        _pickup_y = -0.19644058959803903
        _pickup_z = 0.06689133441078914
        
        self.pickup(_pickup_x, _pickup_y, _pickup_z)
        self.hand_to_user()
        # self.remove(np.pi/2)
        # self.put_in_bowl()
        # self.stir()

def get_mover():
    global mover
    return mover

if __name__ == "__main__":
    try:
        mover = BaxterMover()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MotionCommandsNode shutting down.")