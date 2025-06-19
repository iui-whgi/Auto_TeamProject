#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import datetime
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError

class ArucoWhiteDocking:
    def __init__(self):
        rospy.init_node('aruco_white_docking')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # ArUco 설정
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters_create()

        self.camera_matrix = np.array([[488.5857321, 0, 324.86454285],
                                       [0, 488.93509808, 251.35097507],
                                       [0, 0, 1]])
        self.dist_coeffs = np.array([0.219543, -0.585101, 0.001202, 0.002256, 0.461918])

        # 상태 관리
        self.current_position = None
        self.start_position = None
        self.target_distance = 0.12  # 12cm 전진 목표
        self.start_time = rospy.Time.now()
        self.state = "searching"

        # ArUco P제어 파라미터 (더 부드럽게 조정)
        self.linear_Kp = 0.4  
        self.angular_Kp = 0.8  
        self.max_linear_speed = 0.08  
        self.min_linear_speed = 0.02  
        
        # 회전 속도도 더 부드럽게
        self.max_angular_speed = 0.3  
        self.search_angular_speed = -0.08  
        
        # 오도메트리 모드 파라미터
        self.forward_speed = 0.05  
        self.correction_angular_speed = -0.1  

        # 흰 상자 추적 파라미터
        self.seen_large_square = False  # 큰 사각형을 본 적 있는지
        self.white_approach_speed = 0.1  # 흰 상자 접근 속도

        # 타임아웃 관리 (3초 종료 조건)
        self.last_detection_time = rospy.Time.now()
        self.timeout_duration = rospy.Duration(3.0)  # 3초 타임아웃

        rospy.loginfo("ArucoWhiteDocking initialized.")

    def image_callback(self, msg):
        if self.state == "arrived":
            rospy.loginfo_throttle(2, "[STATE] current: arrived. stopping all actions.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"[cv_bridge error] {e}")
            return

        display_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        twist = Twist()
        detection_found = False  # 이번 프레임에서 무언가 탐지했는지 플래그

        if ids is not None:
            # ArUco 마커가 보이면 P제어로 추적
            detection_found = True
            self.last_detection_time = rospy.Time.now()  # 탐지 시간 업데이트
            
            aruco.drawDetectedMarkers(display_frame, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, 0.16, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(rvecs)):
                aruco.drawAxis(display_frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
            
            x, z = tvecs[0][0][0], tvecs[0][0][2]
            angle = math.atan2(x, z) - 0.12
            distance = math.sqrt(x**2 + z**2)

            speed = self.linear_Kp * distance
            speed = min(max(speed, self.min_linear_speed), self.max_linear_speed)

            # 회전 속도에도 제한 추가
            angular_speed = -self.angular_Kp * angle
            angular_speed = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)

            if abs(angle) < 0.15:  
                twist.linear.x = speed
                twist.angular.z = 0.0
                rospy.loginfo(f"[ARUCO TRACKING] forward to marker (angle={angle:.2f}, distance={distance:.2f})")
            else:
                twist.linear.x = speed * 0.7  # 회전할 때는 전진 속도를 70%로 감소
                twist.angular.z = angular_speed
                rospy.loginfo(f"[ARUCO TRACKING] turning to marker (angle={angle:.2f}, distance={distance:.2f})")

            self.state = "tracking"

        else:
            # ArUco 마커가 안 보이면 흰 정사각형 찾기 (두 번째 코드 로직 사용)
            rospy.loginfo_throttle(2, "[ARUCO LOST] detecting white square...")
            
            # 흰색 임계값 처리 (두 번째 코드와 동일)
            _, thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            squares = []
            for cnt in contours:
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                area = cv2.contourArea(cnt)

                if len(approx) == 4 and area > 20 and cv2.isContourConvex(approx):
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    if 0.7 <= aspect_ratio <= 1.3:  # 두 번째 코드의 비율 사용
                        squares.append((area, approx, w, h))
                        # 초록색 테두리: 후보 정사각형
                        cv2.drawContours(display_frame, [approx], 0, (0, 255, 0), 2)

            if squares:
                # 흰 상자를 찾았으므로 탐지 시간 업데이트
                detection_found = True
                self.last_detection_time = rospy.Time.now()
                
                # 가장 작은 사각형 (가장 가까운) 선택
                _, best_square, w, h = sorted(squares, key=lambda x: x[0])[0]
                x, y, ww, hh = cv2.boundingRect(best_square)
                
                # 파란 사각형 강조 + 텍스트
                cv2.rectangle(display_frame, (x, y), (x+ww, y+hh), (255, 0, 0), 2)
                cv2.putText(display_frame, f"{w}x{h}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # 로그 저장
                now_str = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                log_entry = f"[{now_str}] width={w}, height={h}\n"
                with open("whitebox_log.txt", "a") as f:
                    f.write(log_entry)
                rospy.loginfo(f"[WHITE BOX LOGGED] {log_entry.strip()}")

                # 플래그 설정: 큰 사각형을 본 적 있음
                if w >= 140 or h >= 140:
                    self.seen_large_square = True

                # 도킹 완료 조건 (두 번째 코드 로직)
                if w >= 140 and h >= 140:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    rospy.loginfo("[WHITE BOX] Docking complete - stopping.")
                    self.state = "arrived"
                # 갑자기 작아졌는데 예전에 크게 본 적 있으면 멈춤 (카메라 너무 가까움)
                elif self.seen_large_square and (w < 130 or h < 130):
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    rospy.loginfo("[WHITE BOX] Too close after large seen - stopping.")
                    self.state = "arrived"
                else:
                    # 중앙 정렬 후 접근
                    M = cv2.moments(best_square)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        center = frame.shape[1] // 2

                        if cx < center - 10:
                            twist.angular.z = 0.1
                            twist.linear.x = 0.0
                            rospy.loginfo("[WHITE BOX] rotate left to center")
                        elif cx > center + 10:
                            twist.angular.z = -0.1
                            twist.linear.x = 0.0
                            rospy.loginfo("[WHITE BOX] rotate right to center")
                        else:
                            twist.angular.z = 0.0
                            twist.linear.x = self.white_approach_speed  # 천천히 접근
                            rospy.loginfo("[WHITE BOX] centered → forward approach")
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        rospy.loginfo("[WHITE BOX] moment calculation failed")
            else:
                # 흰 상자도 못 찾으면 오른쪽으로 회전하며 탐색
                twist.angular.z = -0.1
                twist.linear.x = 0.0
                rospy.loginfo("[SEARCH] no marker/square → rotate right")

            self.state = "white_tracking"

        # 3초 타임아웃 체크 (아무것도 탐지하지 못한 경우)
        if not detection_found:
            current_time = rospy.Time.now()
            if current_time - self.last_detection_time > self.timeout_duration:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                rospy.logwarn("[TIMEOUT] No marker/square detected for 3 seconds - stopping.")
                self.state = "arrived"
                
                # 타임아웃 로그 저장
                now_str = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                timeout_log = f"[{now_str}] TIMEOUT: No detection for 3+ seconds - STOPPED\n"
                with open("whitebox_log.txt", "a") as f:
                    f.write(timeout_log)

        # 도착 상태가 아니면 명령 발행
        if self.state != "arrived":
            self.cmd_pub.publish(twist)

        # 타임아웃 정보를 화면에 표시
        current_time = rospy.Time.now()
        time_since_detection = (current_time - self.last_detection_time).to_sec()
        cv2.putText(display_frame, f"No detection: {time_since_detection:.1f}s", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        rospy.loginfo_throttle(2, f"[STATE] current: {self.state}, no_detection_time: {time_since_detection:.1f}s")

        cv2.imshow("Aruco White Docking", display_frame)
        if cv2.waitKey(1) == 27:  # ESC key
            rospy.signal_shutdown("ESC pressed")

    def odom_callback(self, msg):
        # 오도메트리에서 현재 위치 저장
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

if __name__ == '__main__':
    try:
        ArucoWhiteDocking()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()