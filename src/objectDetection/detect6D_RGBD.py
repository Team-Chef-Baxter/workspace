import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2 as rs

model = YOLO('yolov10n.pt')

# Define camera intrinsic parameters (replace with actual values from your camera)
# These can be obtained via camera calibration or from the RealSense SDK
camera_intrinsics = {
    'fx': 500,  # Placeholder value; replace with actual fx
    'fy': 500,  # Placeholder value; replace with actual fy
    'cx': 320,  # Placeholder value; replace with actual cx
    'cy': 240   # Placeholder value; replace with actual cy
}

# Function to convert pixel coordinates and depth to 3D coordinates
def pixel_to_3d(u, v, depth, camera_intrinsics):
    fx, fy, cx, cy = camera_intrinsics['fx'], camera_intrinsics['fy'], camera_intrinsics['cx'], camera_intrinsics['cy']
    Z = depth / 1000.0  # Convert depth from mm to meters
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return np.array([X, Y, Z], dtype=np.float32)

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Configure the pipeline to stream RGB and Depth
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline
pipeline.start(config)

# Retrieve the camera intrinsics from RealSense
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()

camera_intrinsics = {
    'fx': depth_intrinsics.fx,
    'fy': depth_intrinsics.fy,
    'cx': depth_intrinsics.ppx,
    'cy': depth_intrinsics.ppy
}

# Function to estimate pose using solvePnP
def estimate_pose(object_points, image_points, camera_matrix, dist_coeffs):
    success, rotation_vector, translation_vector = cv2.solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs
    )
    if success:
        # Convert rotation vector to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

        # Get roll, pitch, yaw from the rotation matrix
        sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

        return rotation_vector, translation_vector, (roll, pitch, yaw)
    else:
        return None, None, None

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        rgb_image = np.asanyarray(color_frame.get_data())

        # Run object detection on the RGB image
        results = model.predict(rgb_image)

        # Iterate through the detected objects
        for result in results:
            # Depending on the YOLO version, adjust attribute access
            # Here assuming `boxes` attribute with `xyxy` format
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Get the center pixel of the bounding box
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                # Get the depth value at the center pixel
                Z = depth_image[v, u]

                if Z == 0:
                    print("Invalid depth at the center pixel.")
                    continue

                # Calculate the 3D position (X, Y, Z)
                position_3d = pixel_to_3d(u, v, Z, camera_intrinsics)

                # Define object points (3D coordinates in the object's local coordinate system)
                # Replace this with the actual key points of the object you are detecting
                object_points = np.array([
                    [-0.05, -0.05, 0.0],  # Example 3D points (in meters)
                    [0.05, -0.05, 0.0],
                    [0.05, 0.05, 0.0],
                    [-0.05, 0.05, 0.0]
                ], dtype=np.float32)

                # Define corresponding image points (2D coordinates in the image)
                image_points = np.array([
                    [x1, y1],
                    [x2, y1],
                    [x2, y2],
                    [x1, y2]
                ], dtype=np.float32)

                # Camera matrix (intrinsic parameters)
                camera_matrix = np.array([
                    [camera_intrinsics['fx'], 0, camera_intrinsics['cx']],
                    [0, camera_intrinsics['fy'], camera_intrinsics['cy']],
                    [0, 0, 1]
                ], dtype=np.float32)

                # Distortion coefficients (assuming no lens distortion)
                dist_coeffs = np.zeros((4, 1))

                # Estimate pose
                rotation_vector, translation_vector, angles = estimate_pose(
                    object_points, image_points, camera_matrix, dist_coeffs
                )

                if rotation_vector is not None:
                    roll, pitch, yaw = angles

                    # Output 6D pose (position and orientation)
                    print(f"Object Position (X, Y, Z): {position_3d}")
                    print(f"Object Orientation (Roll, Pitch, Yaw): "
                          f"({np.degrees(roll):.2f}, {np.degrees(pitch):.2f}, {np.degrees(yaw):.2f})")
                else:
                    print("Pose estimation failed.")

                # Optionally, draw the bounding box and 3D position on the RGB frame
                cv2.rectangle(rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(rgb_image, (u, v), 5, (0, 0, 255), -1)
                cv2.putText(rgb_image, f"X:{position_3d[0]:.2f} Y:{position_3d[1]:.2f} Z:{position_3d[2]:.2f}",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Display the RGB frame with annotations
        cv2.imshow('Detection with 3D Pose', rgb_image)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
