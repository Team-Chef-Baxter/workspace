import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2 as rs
import open3d as o3d

def main():
    # -----------------------------
    # 1. Initialize RealSense Pipeline
    # -----------------------------
    pipeline = rs.pipeline()
    config = rs.config()

    # Configure the pipeline to stream depth and color frames
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start the pipeline
    pipeline.start(config)

    # Retrieve the camera intrinsics
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()

    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    color_intrinsics = color_profile.get_intrinsics()

    camera_intrinsics = {
        'fx': depth_intrinsics.fx,
        'fy': depth_intrinsics.fy,
        'cx': depth_intrinsics.ppx,
        'cy': depth_intrinsics.ppy,
        'width': depth_intrinsics.width,
        'height': depth_intrinsics.height
    }

    # -----------------------------
    # 2. Load YOLO Model with Segmentation
    # -----------------------------
    # Ensure you have a YOLO model that supports segmentation
    # You can use 'yolov8n-seg.pt' or any other segmentation-capable model
    model = YOLO('yolov8n-seg.pt')  # Replace with your model path

    # -----------------------------
    # 3. Define Utility Functions
    # -----------------------------
    def pixel_to_3d(u, v, depth, intrinsics):
        """
        Convert pixel coordinates and depth to 3D coordinates.
        """
        fx, fy, cx, cy = intrinsics['fx'], intrinsics['fy'], intrinsics['cx'], intrinsics['cy']
        Z = depth / 1000.0  # Convert depth from mm to meters
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        return np.array([X, Y, Z], dtype=np.float32)

    def create_point_cloud(depth_image, color_image, intrinsics):
        """
        Create an Open3D point cloud from depth and color images.
        """
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)),
            o3d.geometry.Image(depth_image),
            convert_rgb_to_intensity=False,
            depth_scale=1000.0,  # Scale depth from mm to meters
            depth_trunc=3.0       # Truncate depth beyond 3 meters
            # Removed convert_rgb_to_bgr=False as it's not a valid argument
        )

        intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
            width=intrinsics['width'],
            height=intrinsics['height'],
            fx=intrinsics['fx'],
            fy=intrinsics['fy'],
            cx=intrinsics['cx'],
            cy=intrinsics['cy']
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            intrinsic_o3d
        )

        # Flip the point cloud for correct orientation
        pcd.transform([[1, 0, 0, 0],
                       [0, -1, 0, 0],
                       [0, 0, -1, 0],
                       [0, 0, 0, 1]])
        return pcd

    # -----------------------------
    # 4. Processing Loop
    # -----------------------------
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
            color_image = np.asanyarray(color_frame.get_data())

            # Run object detection with segmentation
            results = model.predict(color_image, verbose=False)

            # Iterate through detected objects
            for result in results:
                # Ensure the result has masks
                if not hasattr(result, 'masks') or result.masks is None:
                    continue

                # Iterate over each detection
                for i in range(len(result.boxes)):
                    # Extract mask, class, confidence for the current detection
                    try:
                        mask = result.masks.data[i]  # Access the i-th mask tensor
                        cls = result.boxes.cls[i]
                        conf = result.boxes.conf[i]
                        name = result.names[int(cls)]
                    except IndexError:
                        print(f"IndexError: No mask available for detection {i}")
                        continue
                    except Exception as e:
                        print(f"Unexpected error: {e}")
                        continue

                    # Convert mask Tensor to NumPy array
                    # Ensure that mask is a 2D array
                    if mask.dim() == 2:
                        mask_np = mask.cpu().numpy() if hasattr(mask, 'cpu') else mask.numpy()
                    elif mask.dim() == 3:
                        # If mask has multiple channels, take the first one
                        mask_np = mask[0].cpu().numpy() if hasattr(mask, 'cpu') else mask[0].numpy()
                    else:
                        print(f"Unsupported mask dimensions: {mask.dim()}")
                        continue

                    # Thresholding to create binary mask
                    mask_binary = (mask_np > 0.5).astype(np.uint8) * 255  # Binary mask: 0 or 255

                    # Find contours to get bounding box from mask
                    contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        cnt = contours[0]
                        x1, y1, w, h = cv2.boundingRect(cnt)
                        x2, y2 = x1 + w, y1 + h
                    else:
                        # If no contours found, use the existing bounding box from YOLO
                        box = result.boxes[i]
                        x1, y1, x2, y2 = map(int, box.xyxy.cpu().numpy())

                    # Extract the point cloud for the current frame
                    full_pcd = create_point_cloud(depth_image, color_image, camera_intrinsics)

                    # Apply the mask to the point cloud
                    # Convert binary mask to boolean array
                    mask_bool = mask_binary.astype(bool)
                    # Flatten the mask to match the point cloud points
                    mask_flat = mask_bool.flatten()
                    # Extract points and colors within the mask
                    object_points = np.asarray(full_pcd.points)[mask_flat]
                    object_colors = np.asarray(full_pcd.colors)[mask_flat]

                    if object_points.size == 0:
                        print(f"Object: {name} | Centroid: No valid points found within the mask.")
                        continue

                    # Create Open3D point cloud for the object
                    object_pcd_filtered = o3d.geometry.PointCloud()
                    object_pcd_filtered.points = o3d.utility.Vector3dVector(object_points)
                    object_pcd_filtered.colors = o3d.utility.Vector3dVector(object_colors)

                    # Compute the centroid
                    centroid = object_pcd_filtered.get_center()
                    print(f"Object: {name} | Centroid: {centroid}")

                    # Draw bounding box and centroid information on the RGB image
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(color_image, f"{name} ({conf:.2f})", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Annotate centroid information on the image
                    cv2.putText(color_image, f"Centroid: X:{centroid[0]:.2f} Y:{centroid[1]:.2f} Z:{centroid[2]:.2f}m",
                                (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Display the RGB frame with annotations
            cv2.imshow('Detection with 3D Centroid', color_image)

            # Exit on pressing 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by user.")

    finally:
        # Stop the pipeline
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
