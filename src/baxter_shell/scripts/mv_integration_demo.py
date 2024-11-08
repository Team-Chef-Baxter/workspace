#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from shell_demo import Demo

def callback(data):
    # Ensure detections are available
    if data.detections:
        # Using the first detection for demonstration
        detection = data.detections[0]
        
        # Extract x, y, z coordinates from the AprilTag detection message
        x = detection.pose.pose.pose.position.x
        y = detection.pose.pose.pose.position.y
        z = detection.pose.pose.pose.position.z

        # Move Baxter's arm to the detected position
        demo_obj.move_baxter_arm(x, y, z)
    else:
        rospy.loginfo("No AprilTag detections found.")

def listener():
    # Initialize the ROS node
    rospy.init_node('mv_integration_demo', anonymous=True)

    # Initialize the Demo object from shell_demo.py
    global demo_obj
    demo_obj = Demo()

    # Subscribe to the /apriltag_detection topic
    rospy.Subscriber("/apriltag_detection", AprilTagDetectionArray, callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()
