#!/usr/bin/env python3

import sys
print("Python executable being used:", sys.executable)
print("Python version:", sys.version)

from ultralytics import YOLO
import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
from speechrecog_ros.msg import DetectedObject, SaladRecipe, SaladCommand

rospy.init_node('obj_detect_node', anonymous=True)

detected_object_pub = rospy.Publisher('/detected_objects', DetectedObject, queue_size=10000)

model = YOLO('/home/bhl2/workspace/src/object_detect/runs/oreEpochs/weights/best.pt')

def pixel_to_3d(u, v, depth, camera_intrinsics):
    fx, fy, cx, cy = camera_intrinsics['fx'], camera_intrinsics['fy'], camera_intrinsics['cx'], camera_intrinsics['cy']
    Z = depth / 1000.0
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return np.array([X, Y, Z], dtype=np.float32)

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()

camera_intrinsics = {
    'fx': depth_intrinsics.fx,
    'fy': depth_intrinsics.fy,
    'cx': depth_intrinsics.ppx,
    'cy': depth_intrinsics.ppy
}

try:
    rate = rospy.Rate(1)
    rospy.loginfo("Setting up on YOLO.")
    rospy.loginfo("hAAHAH")

    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        rgb_image = np.asanyarray(color_frame.get_data())

        results = model(rgb_image)

        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                class_name = model.names[cls].lower()

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                Z = depth_image[center_y, center_x]
                if Z == 0:
                    rospy.logwarn(f"Invalid depth at center point of bounding box ({center_x}, {center_y}).")
                    continue

                center_3d = pixel_to_3d(center_x, center_y, Z, camera_intrinsics)


                # NEWEST IGNORE x: -0.084725745 y: 0.15678802 z: 0.563
                # Ignore detections near the specified coordinates
                # x: -0.057843298
                # y: 0.14336321
                # z: 0.537
                
                # OLD IGNORE -0.05436968, 0.13374168, 0.456
                target_x, target_y, target_z = -0.08165596, 0.14607877, 0.563
                tolerance = 0.01
                if (abs(center_3d[0] - target_x) <= tolerance and
                    abs(center_3d[1] - target_y) <= tolerance and
                    abs(center_3d[2] - target_z) <= tolerance):
                    rospy.loginfo(f"Detection ignored at coordinates X:{center_3d[0]:.6f}, Y:{center_3d[1]:.6f}, Z:{center_3d[2]:.6f}")
                    continue

                detected_obj = DetectedObject()
                detected_obj.object_class = class_name
                detected_obj.x = center_3d[0]
                detected_obj.y = center_3d[1]
                detected_obj.z = center_3d[2]
                detected_object_pub.publish(detected_obj)
                rospy.loginfo(f"Published detected object: {detected_obj}")

                cv2.rectangle(rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(rgb_image, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(rgb_image, f"{class_name} X:{center_3d[0]:.2f} Y:{center_3d[1]:.2f} Z:{center_3d[2]:.2f}",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.imshow('Detection with 3D Pose', rgb_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

except rospy.ROSInterruptException:
    rospy.loginfo("ROS Interrupt.")

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
