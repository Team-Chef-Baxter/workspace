import rospy
from speechrecog_ros.msg import SaladRecipe, SaladCommand
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import String
from your_package.srv import PickUp, PickUpRequest  # Replace with actual service names

class MotionCommandsNode:
    def __init__(self):
        rospy.init_node('motion_commands')
        
        # Dictionary to store detected objects and their locations
        self.tag_locations = {}

        # Subscribers
        self.recipe_subscriber = rospy.Subscriber('/salad_recipe', SaladRecipe, self.handle_recipe)
        self.tag_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.update_tag_locations)

        # Publisher for errors
        self.error_publisher = rospy.Publisher('/motion_errors', String, queue_size=10)

        rospy.loginfo("MotionCommandsNode initialized.")

    def update_tag_locations(self, msg):
        """Update the tag_locations dictionary with the latest detections."""
        for detection in msg.detections:
            self.tag_locations[detection.id] = msg  # Update with position TODO: change msg to something else

    def handle_recipe(self, recipe_msg):
        """Handle a new salad recipe."""
        rospy.loginfo("Received a salad recipe. Processing...")
        
        for command in recipe_msg.commands:
            motion = command.motion
            obj = command.object

            # Get the location of the object
            location = self.tag_locations.get(obj)
            if not location:
                error_msg = f"Error: Object '{obj}' not found in tag detections."
                rospy.logerr(error_msg)
                self.error_publisher.publish(error_msg)
                return

            # Handle specific motions
            if motion == "pick up":
                success = self.request_pickup_service(location)
            # elif motion == "hand to user":
            #     success = self.request_hand_to_user_service()
            # elif motion == "put in bowl":
            #     success = self.request_put_in_bowl_service(location)
            # elif motion == "mix bowl":
            #     success = self.request_mix_bowl_service()
            # elif motion == "hand bowl to user":
            #     success = self.request_hand_bowl_to_user_service()
            # else:
                rospy.logerr(f"Unknown motion: {motion}")
                continue

            # Check the result of the service call
            if not success:
                error_msg = f"Error executing motion: {motion} on object: {obj}"
                rospy.logerr(error_msg)
                self.error_publisher.publish(error_msg)
                return
        
        rospy.loginfo("Salad recipe completed successfully!")

    def request_pickup_service(self, location):
        """Request the PickUp service."""
        try:
            rospy.wait_for_service('/pickup_service')
            pickup_service = rospy.ServiceProxy('/pickup_service', PickUp)
            req = PickUpRequest()
            req.location = location
            response = pickup_service(req)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def request_hand_to_user_service(self):
        """Placeholder for HandToUser service."""
        rospy.loginfo("Handing to user...")
        return True

    def request_put_in_bowl_service(self, location):
        """Placeholder for PutInBowl service."""
        rospy.loginfo(f"Putting object at {location} in bowl...")
        return True

    def request_mix_bowl_service(self):
        """Placeholder for MixBowl service."""
        rospy.loginfo("Mixing bowl...")
        return True

    def request_hand_bowl_to_user_service(self):
        """Placeholder for HandBowlToUser service."""
        rospy.loginfo("Handing bowl to user...")
        return True

if __name__ == "__main__":
    try:
        MotionCommandsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MotionCommandsNode shutting down.")
