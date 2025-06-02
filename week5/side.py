import cv2
import numpy as np

# 파라미터
camera_matrix = np.array([[490.85, 0, 324.68],
                          [0, 491.20, 249.68],
                          [0, 0, 1]])
dist_coeffs = np.array([0.224651, -0.607437, -0.001577, 0.002322, 0.506671])

# 이미지 로드
img = cv2.imread('captured_20250602_104507.png')
undistorted = cv2.undistort(img, camera_matrix, dist_coeffs)

# 크기 맞추기
h, w = img.shape[:2]
undistorted = cv2.resize(undistorted, (w, h))

# 나란히 붙이기
combined = np.hstack((img, undistorted))

# 표시 및 저장
cv2.imshow('Original (Left) vs Undistorted (Right)', combined)
cv2.imwrite('comparison.png', combined)
cv2.waitKey(0)
cv2.destroyAllWindows()
