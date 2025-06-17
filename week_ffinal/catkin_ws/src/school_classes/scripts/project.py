#!/usr/bin/env python
import numpy as np
import cv2

# Load image
img = cv2.imread('saved_image.jpg')
if img is None:
    raise FileNotFoundError("saved_image.jpg not found.")

# LiDAR 3D points
points_lidar = np.array([
    [-2.095861, 1.418917, 0.0],   # index 93
    [ 0.080836, 0.318915, 0.0],   # index 48
    [ 0.701410, 0.086738, 0.0],   # index 4
    [ 1.441083, -1.481813, 0.0]   # index 201
], dtype=np.float32)

# Intrinsic Camera Matrix
K = np.array([
    [488.5857231, 0.0,           324.86454285],
    [0.0,         488.93509088,  251.35097507],
    [0.0,         0.0,           1.0]
], dtype=np.float64)

# Distortion coefficients
dist_coeffs = np.array([
    0.163996, -0.271840, 0.001056, -0.001666, 0.0
], dtype=np.float64)

# Rotation matrix and translation vector from solvePnP
R = np.array([
    [ 0.86594984,  0.46824683, -0.17571507],
    [-0.11467574, -0.156081,   -0.98106483],
    [-0.48680628,  0.86970318, -0.08146176]
], dtype=np.float64)

t = np.array([
    [-0.22893531],
    [-0.08460995],
    [ 1.78594894]
], dtype=np.float64)

# Convert rotation matrix to rotation vector
rvec, _ = cv2.Rodrigues(R)

# Project 3D points to 2D image plane
image_points, _ = cv2.projectPoints(points_lidar, rvec, t, K, dist_coeffs)

# Draw red dots on projected points
h, w = img.shape[:2]
for idx, pt in enumerate(image_points.reshape(-1, 2)):
    u, v = int(round(pt[0])), int(round(pt[1]))
    if 0 <= u < w and 0 <= v < h:
        cv2.circle(img, (u, v), 5, (0, 0, 255), -1)
        print(f"✅ Point {idx}: ({u}, {v}) in image bounds.")
    else:
        print(f"❌ Point {idx}: ({u}, {v}) out of bounds.")

# Save result
cv2.imwrite('projected_result.jpg', img)
print("✅ projected_result.jpg saved.")
