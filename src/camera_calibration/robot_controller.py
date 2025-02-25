import numpy as np

class RobotController:
    def __init__(self):

        pass

    def move_to_calibration_pose(self, pose_index):
        """
        Move the robot to some known or pre-defined calibration pose.
        For demonstration, I just print.
        """
        print(f"[Robot] Moving to calibration pose index {pose_index}...")

    def get_end_effector_pose_world(self) -> np.ndarray:
        """
        Return the current end-effector pose as a 4x4 transform in the world frame.
        For demonstration, returns the identity matrix; in real usage, query the robot's FK.
        """
        return np.eye(4)
