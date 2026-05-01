import cv2 
import numpy as np 
import subprocess

marker_length = 0.05
camera_matrix = np.array([[2480.0, 0.0, 600.0],
                          [0.0, 2480.0, 800.0],
                          [0.0, 0.0, 1.0]], dtype=np.float32)
dist_coeffs = np.zeros((4,1), dtype=np.float32)

def get_pixel_to_metric_ratio(corners):
    """Calculates meters represented by one pixel at current value of Z"""

    pixel_perimeter = cv2.arcLength(corners[0], True)
    avg_pixel_side = pixel_perimeter / 4.0

    return marker_length / avg_pixel_side

def transform_to_world_frame(target_px, marker_center_px, m_per_px):
    """Converts pixel value to metric units"""

    dx_px = target_px[0] - marker_center_px[0]
    dy_px = target_px[1] - marker_center_px[1]

    dx_m = dx_px * m_per_px
    dy_m = -(dy_px * m_per_px)

    return dx_m, dy_m

image = cv2.imread(r"/home/kshitij_jha/prap-25-26/vision/marker copy.jpeg")
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

corners, ids, _ = detector.detectMarkers(image)

if ids is not None: 
    m_per_px = get_pixel_to_metric_ratio(corners)

    marker_center = np.mean(corners[0][0], axis=0)

    yolo_bbox = [300, 1100, 366, 1250] #results[0].boxes.xyxy

    center_u = (yolo_bbox[0] + yolo_bbox[2])/2
    center_v = (yolo_bbox[1] + yolo_bbox[3])/2
    target_pixel = (center_u, center_v) # (333, 1175)

    rel_x, rel_y = transform_to_world_frame(target_pixel, marker_center, m_per_px)

    x_val = round(float(rel_x), 5)
    y_val = round(float(rel_y), 5)

    ros_cmd = (
        f'ros2 action send_goal /move_pose poker_interfaces/action/MovePose '
        f'"{{x: {x_val}, y: {y_val}, z:0.0, pitch: 0.0, roll: 0.0, duration: 5.0}}"'
    )

    print(f"\n--- Coordinate Information ---")
    print(f"Card Center (Pixels): {target_pixel}")
    print(f"Target (m): X={x_val}, Y={y_val}")
    print(f"\nExecuting this command:\n{ros_cmd}")
    
    # subprocess.run(ros_cmd, shell=True)

else:
    print("Marker not detected. Check image path or lighting.")