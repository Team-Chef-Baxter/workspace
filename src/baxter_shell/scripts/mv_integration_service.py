#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from baxter_shell.srv import PickUp, PickUpResponse  # Adjust based on your package
from shell_demo import Demo

class PickUpServiceNode:
    def __init__(self):
        rospy.init_node('pickup_service_node', anonymous=True)

        # Initialize the Demo object
        self.demo_obj = Demo()

        # Create the service
        self.service = rospy.Service('PickUp', PickUp, self.handle_pickup_request)
        rospy.loginfo("PickUp service is ready.")

    def handle_pickup_request(self, req):
        rospy.loginfo("Received a PickUp request.")

        if not req.detections.detections:
            rospy.logerr("No detections in the request.")
            return PickUpResponse(success=False, message="No detections found.")

        # Process each detection in the array
        for detection in req.detections.detections:
            x = detection.pose.pose.pose.position.x
            y = detection.pose.pose.pose.position.y
            z = detection.pose.pose.pose.position.z

            rospy.loginfo(f"Moving to position: x={x}, y={y}, z={z}")
            try:
                self.demo_obj.move_baxter_arm(x, y, z)  # Assuming move_baxter_arm is implemented
            except Exception as e:
                rospy.logerr(f"Failed to move Baxter arm: {e}")
                return PickUpResponse(success=False, message=f"Error moving to position {x}, {y}, {z}")

        rospy.loginfo("PickUp operation completed successfully.")
        return PickUpResponse(success=True, message="PickUp operation completed successfully.")

if __name__ == '__main__':
    try:
        node = PickUpServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("PickUpServiceNode shutting down.")
