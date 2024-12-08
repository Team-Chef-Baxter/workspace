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
from motion_planning_services.srv import PickUp, PickUpRequest  # Replace with actual service names
from baxter_mover import get_mover

"""
File Breakdown:

Has a master class called BaxterMover

Upon Init, runs a node to republish joint_states so they have a good timestamp.

A helpful note for motion calls: Ignore strawberry/plum at 0.01, 0.16, 0.7, it's the bowl.

See Motion Workflows doc for constraints that I think are imoprtant
- Most Important is you pick something up and the next instruction doesn't use it, please remove
- You have some bookkeeping to do for motion (removed object's locations, above bullet pt)
- I would just have a removed objects map, ie {plum: [0.1, 0.1, 0.1]}

TO RUN:
- roslaunch motion_planning moveit_init.launch
- if you want visualization, run "roslaunch baxter_moveit_config demo.launch" first

Prereqs:
- Turn on Baxter
- hit f1 on upside down letters
- wait until letters on screen are blue (not saying loading)
- ./baxter.sh
- source /opt/ros/noetic/setup.bash
- source devel_isolated/setup.bash
- E-Stop button not engaged
- rosrun baxter_tools enable_robot.py -e
-

pickup(x, y, z): Moves arm to a little above target, then descends, then ascends
    Recommendation for Integration: Do PickUp(), then check object position we tried to pickup
    We always move 0.2 above where we grabbed, so check that distance. See helper functions.

remove(Angle): Moves arm to position around bowl at angle, slowly, hoping to brush off object
    Returns a point [x, y, z] of where we think we probably left object
    Recommendation for Integration: Check if object is still really close to gripper afterwards
    Recommendation for Integration: Have like 3 angles around bowl, one for each object

Shove(Start_pt, End_pt): [NOT DONE YET]
    Inputs: Two 3 dimensional arrays
    Why: When we remove an object we have to get it back to a pickup position somehow
    Usage: When you do remove(), object position becomes tricky. Use remove()'s return, and shove
    away from bowl. I provide example code for 3 angles (front, left, right of bowl) and how to 
    shove objects back to usable spot.

    Effect: Moves arm from start to end with constant z val

    Verify it works with vision

stir(): Hardcoded motion to move right arm to the side and stir around bowl with spoon in left

hand_to_user(): Moves arm to position at 'left' (Baxter POV) end of table to give item to user
    Recommendation for Integration: Check that object is in hand at end of this, 
    otherwise we failed

put_in_bowl(): Moves arm into bowl, moves to side hoping to drop object in. Assumes object in
hand. Moves arm back into air. Verify object is not in hand, and not visible outside of bowl.

neutral(): Puts both arms in air.

See video titled Baxter_Motions.mp4 in drive for real examples of all of these. 

Helper Functions:

    These are independent from rest of codebase and could be copied to other files. 

    transform_pt([x, y, z]): Transforms a point in camera POV to robot POV 
    Useful for verifying a successful pickup. 

    



"""
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

mover = None # for instance of BaxterMover

# Dictionary to store detected objects and their locations
tag_locations = {} # object_class : (x, y, z)

# if __name__ == '__main__':
#     mover = BaxterMover()
#     mover.demo()
#     rospy.spin()

def get_tag_locations():
    global tag_locations
    return tag_locations

def handle_hand_to_user(req):
    rospy.loginfo("Handling hand to user by service.")
    success = mover.hand_to_user()
    return success

class MotionCommandsNode:
    def __init__(self):
        rospy.init_node('motion_commands')

        # Motion planner
        mover = get_mover()
        self.active = False

        # Subscribers
        self.recipe_subscriber = rospy.Subscriber('/salad_recipe', SaladRecipe, self.handle_recipe)
        self.tag_subscriber = rospy.Subscriber('/detected_objects', DetectedObject, self.update_locations)

        # Publisher for errors
        self.error_publisher = rospy.Publisher('/motion_errors', String, queue_size=10)
        
        rospy.loginfo("MotionCommandsNode initialized.")

    def update_locations(self, msg):
        global tag_locations
        """Update the tag_locations dictionary with the latest detections."""
        rospy.loginfo(f"IN UPDATE LOCATIONS. Adding new location for {msg.object_class}")
        tag_locations[msg.object_class] = (msg.x, msg.y, msg.z)


    def handle_recipe(self, recipe_msg):
        global tag_locations
        """Handle a new salad recipe."""
        rospy.loginfo("IN HANDLE RECIPE. Processing...")
        
        for command in recipe_msg.commands:
            motion = command.motion
            obj = command.object

            # Get the location of the object
            location = tag_locations.get(obj)
            if motion != "remove from hand" and motion != "stir" and not location:
                error_msg = f"Error: Object '{obj}' not found in tag detections."
                rospy.logerr(error_msg)
                self.error_publisher.publish(error_msg)
                return

            # Handle specific motions
            if motion == "pick up":
                service_name = '/pickup_service'
                rospy.loginfo(f"Waiting for the service {service_name}...")
                rospy.wait_for_service(service_name)
                rospy.loginfo("Service is now available.")

                # Create the service proxy
                pickup_service = rospy.ServiceProxy(service_name, PickUp)

                # Create and populate the request
                request = PickUpRequest()
                request.object_class = obj
                request.x = location[0]
                request.y = location[1]
                request.z = location[2]

                # Call the service
                rospy.loginfo("Sending request to the pickup service...")
                response = pickup_service(request)

                # Check the response
                rospy.loginfo(f"Pick up service response: success={response.success}")
                if (response.success == False):
                    rospy.logerr("PICK UP FAILED. Exiting recipe...")
                    error_msg = f"Oops! There was an error in picking up. I couldn't finish the recipe."
                    self.error_publisher.publish(error_msg)
                    break

            elif motion == "hand to user":
                service_name = '/hand_to_user_service'
                rospy.loginfo(f"Waiting for the service {service_name}...")
                rospy.wait_for_service(service_name)
                rospy.loginfo("Service is now available.")

                # Create the service proxy
                hand_to_user_service = rospy.ServiceProxy(service_name, HandToUser)

                # Create and populate the request
                request = HandToUserRequest()

                # Call the service
                rospy.loginfo("Sending request to the hand to user service...")
                response = hand_to_user_service(request)

                # Check the response
                rospy.loginfo(f"Hand to user service response: success={response.success}")
                if (response.success == False):
                    rospy.logerr("HAND TO USER FAILED. Exiting recipe...")
                    error_msg = f"Oops! There was an error in handing to user. I couldn't finish the recipe."
                    self.error_publisher.publish(error_msg)
                    break
                
            elif motion == "put in bowl":
                service_name = '/put_in_bowl_service'
                rospy.loginfo(f"Waiting for the service {service_name}...")
                rospy.wait_for_service(service_name)
                rospy.loginfo("Service is now available.")

                # Create the service proxy
                put_in_bowl_service = rospy.ServiceProxy(service_name, PutInBowl)

                # Create and populate the request
                request = PutInBowlRequest()

                # Call the service
                rospy.loginfo("Sending request to the put in bowl service...")
                response = put_in_bowl_service(request)

                # Check the response
                rospy.loginfo(f"Put in bowl service response: success={response.success}")
                if (response.success == False):
                    rospy.logerr("PUT IN BOWL FAILED. Exiting recipe...")
                    error_msg = f"Oops! There was an error in putting in bowl. I couldn't finish the recipe."
                    self.error_publisher.publish(error_msg)
                    break
                
            elif motion == "stir":
                service_name = '/stir_service'
                rospy.loginfo(f"Waiting for the service {service_name}...")
                rospy.wait_for_service(service_name)
                rospy.loginfo("Service is now available.")

                # Create the service proxy
                stir_service = rospy.ServiceProxy(service_name, Stir)

                # Create and populate the request
                request = StirRequest()

                # Call the service
                rospy.loginfo("Sending request to the stir service...")
                response = stir_service(request)

                # Check the response
                rospy.loginfo(f"Stir service response: success={response.success}")
                if (response.success == False):
                    rospy.logerr("STIR FAILED. Exiting recipe...")
                    error_msg = f"Oops! There was an error in stirring. I couldn't finish the recipe."
                    self.error_publisher.publish(error_msg)
                    break
                
            else:
                rospy.logerr(f"Unknown motion: {motion}. Exiting recipe...")
                error_msg = f"Oops! There was an error in understanding the recipe instructions. I couldn't finish the recipe."
                self.error_publisher.publish(error_msg)
                break

            # Check the result of the service call
            # if not success:
            #     error_msg = f"Error executing motion: {motion} on object: {obj}"
            #     rospy.logerr(error_msg)
            #     self.error_publisher.publish(error_msg)
            #     return
        
        rospy.loginfo("Salad recipe completed successfully!")

if __name__ == "__main__":
    try:
        MotionCommandsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MotionCommandsNode shutting down.")