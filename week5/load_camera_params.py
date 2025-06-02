import numpy as np
import cv2

# 저장된 보정 파라미터 불러오기
def load_camera_parameters(calibration_file='camera_calibration.npz'):
    """
    저장된 카메라 보정 파라미터를 불러오는 함수
    """
    try:
        # 보정 데이터 로드
        calib_data = np.load(calibration_file)
        
        camera_matrix = calib_data['camera_matrix']
        dist_coeffs = calib_data['dist_coeffs']
        image_shape = calib_data['image_shape']
        reprojection_error = calib_data['reprojection_error']
        
        print("=== 카메라 파라미터 ===")
        print(f"이미지 크기: {image_shape}")
        print(f"재투영 오차: {reprojection_error:.3f} 픽셀")
        print()
        
        # 카메라 행렬 파라미터 추출
        fx = camera_matrix[0, 0]  # x축 초점거리
        fy = camera_matrix[1, 1]  # y축 초점거리
        cx = camera_matrix[0, 2]  # 주점 x 좌표
        cy = camera_matrix[1, 2]  # 주점 y 좌표
        
        print("=== 내부 파라미터 (Intrinsic Parameters) ===")
        print("카메라 행렬:")
        print(camera_matrix)
        print()
        print(f"초점거리 fx: {fx:.2f} 픽셀")
        print(f"초점거리 fy: {fy:.2f} 픽셀")
        print(f"주점 cx: {cx:.2f} 픽셀")
        print(f"주점 cy: {cy:.2f} 픽셀")
        print()
        
        # 왜곡 계수
        k1, k2, p1, p2, k3 = dist_coeffs.ravel()
        print("=== 왜곡 계수 (Distortion Coefficients) ===")
        print(f"방사형 왜곡 k1: {k1:.6f}")
        print(f"방사형 왜곡 k2: {k2:.6f}")
        print(f"접선형 왜곡 p1: {p1:.6f}")
        print(f"접선형 왜곡 p2: {p2:.6f}")
        print(f"방사형 왜곡 k3: {k3:.6f}")
        print()
        
        return camera_matrix, dist_coeffs, image_shape
        
    except FileNotFoundError:
        print(f"오류: '{calibration_file}' 파일을 찾을 수 없습니다.")
        print("먼저 카메라 보정을 실행해주세요.")
        return None, None, None
    except Exception as e:
        print(f"파라미터 로드 중 오류 발생: {e}")
        return None, None, None

# 실시간 카메라에서 파라미터 적용
def apply_camera_correction():
    """
    실시간 카메라에 왜곡 보정 적용
    """
    # 저장된 파라미터 불러오기
    camera_matrix, dist_coeffs, image_shape = load_camera_parameters()
    
    if camera_matrix is None:
        return
    
    # 카메라 초기화
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return
    
    print("카메라 실행 중... 'q'를 눌러 종료")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 왜곡 보정 적용
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, (w, h), 1, (w, h))
        
        # 왜곡 보정된 이미지
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, newcameramtx)
        
        # 원본과 보정된 이미지 나란히 표시
        combined = np.hstack((frame, undistorted))
        
        # 텍스트 추가
        cv2.putText(combined, 'Original', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(combined, 'Undistorted', (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('Camera Calibration Result', combined)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

# 이미지 파일에 왜곡 보정 적용
def correct_image(image_path, output_path='corrected_image.jpg'):
    """
    이미지 파일에 왜곡 보정 적용
    """
    # 저장된 파라미터 불러오기
    camera_matrix, dist_coeffs, image_shape = load_camera_parameters()
    
    if camera_matrix is None:
        return
    
    # 이미지 로드
    img = cv2.imread(image_path)
    if img is None:
        print(f"이미지를 불러올 수 없습니다: {image_path}")
        return
    
    # 왜곡 보정 적용
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    
    # 왜곡 보정
    undistorted = cv2.undistort(img, camera_matrix, dist_coeffs, None, newcameramtx)
    
    # 결과 저장
    cv2.imwrite(output_path, undistorted)
    print(f"보정된 이미지가 저장되었습니다: {output_path}")
    
    # 결과 비교 표시
    combined = np.hstack((img, undistorted))
    cv2.imshow('Before vs After', combined)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# 파라미터를 다른 형태로 변환
def convert_parameters():
    """
    카메라 파라미터를 다른 형태로 변환
    """
    camera_matrix, dist_coeffs, image_shape = load_camera_parameters()
    
    if camera_matrix is None:
        return
    
    # 픽셀 단위를 mm 단위로 변환 (예시)
    # 실제 센서 크기가 필요함 (예: APS-C 센서: 23.6mm x 15.6mm)
    sensor_width_mm = 23.6  # 실제 센서 너비 (mm)
    image_width_px = image_shape[0]  # 이미지 너비 (픽셀)
    
    fx_px = camera_matrix[0, 0]
    fx_mm = fx_px * sensor_width_mm / image_width_px
    
    print("=== 파라미터 변환 ===")
    print(f"초점거리 (픽셀): {fx_px:.2f}")
    print(f"초점거리 (mm): {fx_mm:.2f}")
    print(f"화각 (도): {2 * np.arctan(image_width_px / (2 * fx_px)) * 180 / np.pi:.1f}")

# 메인 실행 함수
if __name__ == "__main__":
    print("=== 카메라 보정 파라미터 활용 ===")
    print("1. 저장된 파라미터 불러오기")
    
    # 파라미터 불러오기
    camera_matrix, dist_coeffs, image_shape = load_camera_parameters()
    
    if camera_matrix is not None:
        print("\n사용 가능한 기능:")
        print("- apply_camera_correction(): 실시간 카메라 보정")
        print("- correct_image('image.jpg'): 이미지 파일 보정")
        print("- convert_parameters(): 파라미터 변환")
        
        # 사용 예시
        # apply_camera_correction()  # 실시간 카메라 보정
        # correct_image('test_image.jpg')  # 이미지 파일 보정
        # convert_parameters()  # 파라미터 변환