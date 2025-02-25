import numpy as np
import cv2

def compute_rigid_transform(camera_points: np.ndarray, world_points: np.ndarray) -> np.ndarray:
    """
    Given two sets of corresponding 3D points (camera_points, world_points),
    compute the best-fit rigid transform (rotation + translation) that maps
    camera_points to world_points via SVD.

    """
    #Compute centroids
    centroid_cam = np.mean(camera_points, axis=0)
    centroid_wrd = np.mean(world_points, axis=0)

    #Center the points
    cam_centered = camera_points - centroid_cam
    wrd_centered = world_points  - centroid_wrd

    #Compute covariance
    H = cam_centered.T @ wrd_centered  # shape (3, 3)

    #SVD
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    #Fix reflection if needed
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    #Translation
    t = centroid_wrd - R @ centroid_cam

    # Build 4x4
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = t
    return T


def generate_checkerboard_points(board_size, square_size):
    """
    Generate 3D coordinates (x, y, 0) of the checkerboard corners in the
    checkerboard's LOCAL coordinate system (z=0 plane).
    """
    nx, ny = board_size
    obj_points = []
    for j in range(ny):
        for i in range(nx):
            x = i * square_size
            y = j * square_size
            obj_points.append([x, y, 0.0])
    return np.array(obj_points, dtype=np.float32)


def calibrate_camera_pose(
    robot, 
    camera, 
    board_size=(6,5), 
    square_size=0.02,  
    T_object_in_ee=np.eye(4), 
    num_poses=5
):
    """
    Perform camera extrinsic calibration using a checkerboard approach and the robot's
    forward kinematics.
    """
    #Generate local 3D corner positions in the checkerboard's coordinate frame
    local_corners_3d = generate_checkerboard_points(board_size, square_size)
    num_corners = len(local_corners_3d)

    all_cam_points_3d = []
    all_wrd_points_3d = []

    for i in range(num_poses):
        print(f"[Calibration] Moving robot to pose {i+1}/{num_poses}...")

        robot.move_to_calibration_pose(i)
        T_ee_world = robot.get_end_effector_pose_world()
        T_obj_world = T_ee_world @ T_object_in_ee
        color_img = camera.capture_color_image()
        if color_img is None:
            print("ERROR: No color image captured.")
            continue

        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        ret, corners_2d = cv2.findChessboardCorners(gray, board_size, None)

        if not ret:
            print(f"WARNING: Checkerboard not found at pose {i+1}. Skipping.")
            continue

        corners_2d = corners_2d.reshape(-1, 2)  
        cam_corners_3d = []
        for (u, v) in corners_2d:
            u_int, v_int = int(round(u)), int(round(v))
            p_cam = camera.get_3d_point(u_int, v_int)  
            cam_corners_3d.append(p_cam)
        cam_corners_3d = np.array(cam_corners_3d, dtype=np.float32)

        #Convert local checkerboard corners to world coords
        wrd_corners_3d = []
        for corner_local_3d in local_corners_3d:
            corner_local_hom = np.array([corner_local_3d[0],
                                         corner_local_3d[1],
                                         corner_local_3d[2], 1.0])
            corner_world_hom = T_obj_world @ corner_local_hom
            wrd_corners_3d.append(corner_world_hom[:3])
        wrd_corners_3d = np.array(wrd_corners_3d, dtype=np.float32)

        if len(cam_corners_3d) == num_corners:
            all_cam_points_3d.append(cam_corners_3d)
            all_wrd_points_3d.append(wrd_corners_3d)
        else:
            print(f"WARNING: corner count mismatch at pose {i+1}")

    #Combine all data
    if not all_cam_points_3d:
        raise ValueError("No valid calibration data collected. Checkerboard detection failed in all poses.")

    cam_points_conc = np.concatenate(all_cam_points_3d, axis=0)
    wrd_points_conc = np.concatenate(all_wrd_points_3d, axis=0)

    T_camera_world = compute_rigid_transform(cam_points_conc, wrd_points_conc)
    print("[Calibration] Done! T_camera_world =\n", T_camera_world)

    return T_camera_world
