from robot_controller import RobotController
from camera_interface import CameraInterface
from calibration import calibrate_camera_pose
import numpy as np

def main():
    robot = RobotController()
    camera = CameraInterface()


    T_object_in_ee = np.eye(4)

    #Run calibration
    T_cam_world = calibrate_camera_pose(
        robot=robot,
        camera=camera,
        board_size=(6, 5),   # e.g., a 6x5 interior corner pattern
        square_size=0.02,    # 20mm squares
        T_object_in_ee=T_object_in_ee,
        num_poses=5
    )

    print("\n[MAIN] Final camera->world transform:\n", T_cam_world)

if __name__ == "__main__":
    main()
