#!/usr/bin/env python3
import numpy as np
import cv2

# LiDAR 3D points
object_points = np.array([
    [-2.095861, 1.418917, 0.0],   # index 93
    [ 0.080836, 0.318915, 0.0],   # index 48
    [ 0.701410, 0.086738, 0.0],   # index 4
    [ 1.441083, -1.481813, 0.0]   # index 201
], dtype=np.float32)

# 2D image points from camera
image_points = np.array([
    [150, 245],  # index 93
    [336, 213],  # index 48
    [454, 195],  # index 4
    [581, 238]   # index 201
], dtype=np.float32)

# Camera intrinsic matrix
K = np.array([
    [488.5857231, 0.0,           324.86454285],
    [0.0,         488.93509088,  251.35097507],
    [0.0,         0.0,           1.0]
], dtype=np.float64)

# Distortion coefficients
dist_coeffs = np.array([
    0.163996, -0.271840, 0.001056, -0.001666, 0.0
], dtype=np.float64)

# Solve PnP
success, rvec, tvec = cv2.solvePnP(object_points, image_points, K, dist_coeffs)
R, _ = cv2.Rodrigues(rvec)

print("Rotation matrix R:\n", R)
print("Translation vector t:\n", tvec)
