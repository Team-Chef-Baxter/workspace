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

# def create_dummy_apriltag_detection():
#     """Create a dummy AprilTagDetection."""
#     detection = AprilTagDetection()
#     detection.id = [1]  # Example: Tag ID 1
#     detection.size = [0.05]  # Example: Tag size 5 cm

#     # Create a dummy pose
#     pose = PoseWithCovarianceStamped()
#     pose.header.stamp = rospy.Time.now()
#     pose.header.frame_id = "camera_frame"
#     pose.pose.pose.position.x = 1.0
#     pose.pose.pose.position.y = 2.0
#     pose.pose.pose.position.z = 3.0
#     pose.pose.pose.orientation.x = 0.0
#     pose.pose.pose.orientation.y = 0.0
#     pose.pose.pose.orientation.z = 0.0
#     pose.pose.pose.orientation.w = 1.0

#     detection.pose = pose
#     return detection

# def create_dummy_apriltag_detection_array():
#     """Create a dummy AprilTagDetectionArray."""
#     detection_array = AprilTagDetectionArray()
#     detection_array.header = Header()
#     detection_array.header.stamp = rospy.Time.now()
#     detection_array.header.frame_id = "camera_frame"

#     # Add one or more detections
#     detection_array.detections = [create_dummy_apriltag_detection()]
#     return detection_array

def request_pickup_service(detection_array):
    """Request the PickUp service with the given AprilTagDetectionArray."""
    service_name = '/pickup_service'
    
    try:
        rospy.loginfo(f"Waiting for the service {service_name}...")
        rospy.wait_for_service(service_name)
        rospy.loginfo("Service is now available.")

        # Create the service proxy
        pickup_service = rospy.ServiceProxy(service_name, PickUp)

        # Create and populate the request
        request = PickUpRequest()
        # request.detections = detection_array

        # Call the service
        rospy.loginfo("Sending request to the pickup service...")
        response = pickup_service(request)

        # Return the response
        rospy.loginfo(f"Service response: success={response.success}, message='{response.message}'")
        return response.success, response.message
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False, str(e)

if __name__ == '__main__':
    rospy.init_node('pickup_service_client')

    # Create a dummy detection array
    dummy_detection_array = None

    # Call the service
    success, message = request_pickup_service(dummy_detection_array)
    if success:
        rospy.loginfo("PickUp request succeeded.")
    else:
        rospy.loginfo(f"PickUp request failed: {message}")
