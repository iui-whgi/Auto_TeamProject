import cv2
import numpy as np
import glob
import os

# 보정 패턴 설정 - 실제 체스보드에 맞게 조정 필요
pattern_size = (8, 6)  # 내부 코너 수 (가로, 세로) - 실제 패턴에 맞게 수정하세요
square_size = 25  # 체스보드 한 칸의 실제 크기 (mm) - 실제 크기에 맞게 수정하세요

# 객체 점 준비
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size

# 점들을 저장할 배열
objpoints = []  # 3D 실제 점들
imgpoints = []  # 2D 이미지 점들

# 이미지 경로 설정 (~/Downloads/image 폴더에서 이미지 파일들 찾기)
image_folder = os.path.expanduser('~/image')
image_paths = []

# 다양한 이미지 확장자로 파일들 찾기
for ext in ['*.jpg', '*.jpeg', '*.png', '*.bmp']:
    pattern = os.path.join(image_folder, ext)
    image_paths.extend(glob.glob(pattern))

# 파일명으로 정렬
image_paths.sort()

if not image_paths:
    print("에러: 보정용 이미지 파일을 찾을 수 없습니다.")
    print("현재 디렉토리에 jpg, png 등의 이미지 파일이 있는지 확인해주세요.")
    exit()

print(f"발견된 이미지 파일 수: {len(image_paths)}")

# 이미지 처리
successful_images = 0
img_shape = None

for i, fname in enumerate(image_paths):
    print(f"처리 중: {fname} ({i+1}/{len(image_paths)})")
    
    img = cv2.imread(fname)
    if img is None:
        print(f"경고: {fname}을 읽을 수 없습니다.")
        continue
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 이미지 크기 저장 (첫 번째 성공한 이미지 기준)
    if img_shape is None:
        img_shape = gray.shape[::-1]  # (width, height)
    
    # 체스보드 코너 찾기
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    
    if ret:
        print(f"  ✓ 코너 검출 성공")
        objpoints.append(objp)
        
        # 코너 위치 정밀화
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        successful_images += 1
        
        # 결과 시각화 (선택사항)
        img_with_corners = cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
        cv2.imwrite(f'corners_{i}.jpg', img_with_corners)
    else:
        print(f"  ✗ 코너 검출 실패")

print(f"\n성공적으로 처리된 이미지: {successful_images}개")

# 최소 필요 이미지 수 확인
if successful_images < 10:
    print("에러: 보정에 필요한 최소 이미지 수(10개)가 부족합니다.")
    print("체스보드가 명확히 보이는 이미지를 더 추가해주세요.")
    exit()

if img_shape is None:
    print("에러: 처리할 수 있는 이미지가 없습니다.")
    exit()

print("카메라 보정을 시작합니다...")

# 카메라 보정 수행
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_shape, None, None)

if ret:
    print("✓ 카메라 보정 완료!")
    
    # 재투영 오차 계산
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], 
                                          camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    
    mean_error = mean_error/len(objpoints)
    print(f"평균 재투영 오차: {mean_error:.3f} 픽셀")
    
    # 결과 출력
    print("\n=== 보정 결과 ===")
    print("카메라 행렬:")
    print(camera_matrix)
    print("\n왜곡 계수:")
    print(dist_coeffs.ravel())
    
    # 결과 저장
    np.savez('camera_calibration.npz', 
             camera_matrix=camera_matrix,
             dist_coeffs=dist_coeffs,
             rvecs=rvecs,
             tvecs=tvecs,
             image_shape=img_shape,
             reprojection_error=mean_error)
    
    print("\n보정 결과가 'camera_calibration.npz' 파일에 저장되었습니다.")
    
    # 테스트 이미지가 있다면 왜곡 보정 적용
    if image_paths:
        test_img = cv2.imread(image_paths[0])
        if test_img is not None:
            h, w = test_img.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeffs, (w,h), 1, (w,h))
            
            # 왜곡 보정
            dst = cv2.undistort(test_img, camera_matrix, dist_coeffs, None, newcameramtx)
            
            # 결과 저장
            cv2.imwrite('original.jpg', test_img)
            cv2.imwrite('undistorted.jpg', dst)
            print("테스트 이미지의 왜곡 보정 결과:")
            print("  - 원본: original.jpg")
            print("  - 보정: undistorted.jpg")
    
    # 보정 품질 평가
    if mean_error < 0.5:
        print("\n✓ 우수한 보정 품질 (재투영 오차 < 0.5)")
    elif mean_error < 1.0:
        print("\n△ 양호한 보정 품질 (재투영 오차 < 1.0)")
    else:
        print("\n✗ 보정 품질 개선 필요 (재투영 오차 >= 1.0)")
        print("  - 더 많은 이미지 추가")
        print("  - 체스보드 패턴 품질 확인")
        print("  - 조명 조건 개선")

else:
    print("✗ 카메라 보정 실패")
    print("체스보드 패턴과 이미지 품질을 확인해주세요.")