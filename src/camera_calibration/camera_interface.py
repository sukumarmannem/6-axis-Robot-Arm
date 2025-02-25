import cv2
import numpy as np

class CameraInterface:
    def __init__(self):
        pass

    def capture_color_image(self):
        """
        Capture a color image from the camera.
        Return a BGR image (H,W,3) as a numpy array.
        In a real camera, open a stream or read a frame from the device.
        """
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def get_3d_point(self, u, v):
        """
        Given pixel (u,v), return the 3D coordinates [X, Y, Z] in the camera frame.
        This typically requires:
          - A depth image or point cloud aligned with the color image
          - Camera intrinsics for back-projection, or direct lookup in a point cloud
        Here, I just return a dummy point for demonstration.
        """
        Z = 1.0
        X = (u - 320) * 0.001 
        Y = (v - 240) * 0.001
        return np.array([X, Y, Z], dtype=float)
