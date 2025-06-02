import cv2
import numpy as np

# 예시 파라미터
camera_matrix = np.array([[490.85, 0, 324.68],
                          [0, 491.20, 249.68],
                          [0, 0, 1]])
dist_coeffs = np.array([0.224651, -0.607437, -0.001577, 0.002322, 0.506671])

img = cv2.imread('captured_20250602_104507.png')
undistorted = cv2.undistort(img, camera_matrix, dist_coeffs)

cv2.imwrite('undistorted_image.png', undistorted)
cv2.imshow('Undistorted Image', undistorted)
cv2.waitKey(0)
