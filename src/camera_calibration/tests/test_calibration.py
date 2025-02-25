import unittest
import numpy as np
from camera_calibration.calibration import compute_rigid_transform

class TestCalibration(unittest.TestCase):
    def test_compute_rigid_transform(self):
        #Generate some random camera points
        np.random.seed(42)
        camera_points = np.random.rand(10, 3)

        #Define a "true" transformation
        angle = np.deg2rad(30)
        R_true = np.array([
            [ np.cos(angle), -np.sin(angle), 0],
            [ np.sin(angle),  np.cos(angle), 0],
            [ 0,             0,              1]
        ], dtype=float)
        t_true = np.array([1, 2, 3], dtype=float)

        T_true = np.eye(4)
        T_true[:3, :3] = R_true
        T_true[:3, 3]  = t_true

        camera_points_hom = np.hstack([camera_points, np.ones((10,1))])
        world_points_hom  = (T_true @ camera_points_hom.T).T
        world_points      = world_points_hom[:,:3]

        T_est = compute_rigid_transform(camera_points, world_points)

        print("\nEstimated T_camera_world (SVD result):\n", T_est)
        R_est = T_est[:3,:3]
        t_est = T_est[:3,3]
        np.testing.assert_allclose(R_est, R_true, atol=1e-7)
        np.testing.assert_allclose(t_est, t_true, atol=1e-7)

        print("Test passed. Transform recovered successfully.")

if __name__ == "__main__":
    unittest.main()
