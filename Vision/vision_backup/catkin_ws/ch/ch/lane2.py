#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RoadLaneDetector:
    def __init__(self):
        self.bridge = CvBridge()

        # ROS Subscriber & Publisher 설정
        self.image_sub = rospy.Subscriber("img_topic", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/lane_raw", Image, queue_size=1)

        # ROI 4개 점 (정규화 좌표: (0,0) ~ (1,1))
        self.roi_upper_left = (0.25, 0.6)
        self.roi_upper_right = (0.75, 0.6)
        self.roi_lower_right = (1.0, 1.0)
        self.roi_lower_left = (0.0, 1.0)
        
        # 차량 중앙 기준 정규화 좌표 (0.55, 0.95)
        self.ref_norm = (0.55, 0.95)
        # 픽셀 차이 기반 간단 곡률 판단 임계치 (소실점과 기준점 수평 차이)
        self.pixel_threshold = 10
        
        # 노란색 박스 허용 범위: 차량 중앙 기준 좌우 ±20픽셀, 소실점 기준 상하 ±10픽셀
        self.box_horiz_tolerance = 20
        self.box_vert_tolerance = 10

        # 업로드하신 내부 파라미터 파일에서 추출한 값 (실제 값에 맞게 조정)
        self.K = np.array([[718.8560,   0, 607.1928],
                           [      0, 718.8560, 185.2157],
                           [      0,       0,       1]], dtype=np.float32)
        self.dist = np.array([-0.171, 0.024, 0, 0, 0], dtype=np.float32)
        
        # 카메라 외부 정보: 차량 높이 1m, 피치 10° (정면 기준 10° 아래)
        self.camera_height = 1.0  # m
        self.pitch_deg = 10

        # 차량 wheelbase (예: 2.7m)
        self.wheelbase = 1.5

        self.computeHomography()

    def computeHomography(self):
        """ 카메라 내부 파라미터와 외부 정보(높이, 피치)를 사용해
            이미지 좌표 -> 도로 평면(실제 좌표) 변환을 위한 호모그래피 H를 구하고, 그 역행렬 H_inv를 저장
        """
        pitch = np.deg2rad(self.pitch_deg)
        h = self.camera_height
        # 피치 회전 행렬 (X축 회전)
        R = np.array([[1, 0, 0],
                      [0, np.cos(pitch), -np.sin(pitch)],
                      [0, np.sin(pitch),  np.cos(pitch)]])
        # 평행 이동: 카메라가 도로면에서 h만큼 위에 있다고 가정 (도로면 z=0)
        t = np.array([[0], [-h], [0]])
        # 도로 평면 상의 점에 대해, H_ground = [r1, r2, t] (여기서 r1, r2는 R의 첫 두 열)
        H_ground = np.hstack((R[:, 0:2], t))
        H = self.K @ H_ground  # 3x3 행렬
        self.H_inv = np.linalg.inv(H)

    def transformPoint(self, point):
        """ 이미지 좌표 (u, v)를 호모그래피 역변환을 통해 실제 좌표 (x, y; m 단위)로 변환 """
        u, v = point
        pt = np.array([u, v, 1.0])
        world_pt = self.H_inv @ pt
        world_pt /= world_pt[2]
        return world_pt[0:2]  # (x, y)

    def gatherLanePoints(self, lane_lines):
        """ 한쪽 차선에 검출된 모든 선(line)의 양 끝점을 모아 numpy array로 반환 """
        points = []
        for line in lane_lines:
            x1, y1, x2, y2 = line[0]
            points.append([x1, y1])
            points.append([x2, y2])
        return np.array(points)

    def computeLaneCurvature(self, lane_points):
        """ 이미지상의 차선 포인트(lane_points)를 실제 좌표로 변환한 후,
            2차 다항식( x = A*y^2 + B*y + C )으로 적합하고,
            차량 바로 앞쪽(가장 큰 y값)에서의 곡률 반경 R (m 단위)를 계산.
            곡률 공식: R = ((1 + (2*A*y + B)^2)^(3/2)) / |2*A|
        """
        world_points = []
        for pt in lane_points:
            wp = self.transformPoint(pt)
            world_points.append(wp)
        world_points = np.array(world_points)  # shape (N, 2): [:,0]=x, [:,1]=y
        if len(world_points) < 3:
            return None, None  # 충분한 점이 없으면 계산 불가
        # x = f(y)로 가정하여 2차 다항식 적합
        y = world_points[:, 1]
        x = world_points[:, 0]
        poly = np.polyfit(y, x, 2)
        A, B, C = poly
        # 평가 위치: 차량 바로 앞쪽 (가장 큰 y값)
        y_eval = np.max(y)
        curvature = ((1 + (2*A*y_eval + B)**2)**1.5) / np.abs(2*A)
        return curvature, poly

    def computeSteeringAngle(self, left_curv, right_curv, curve_info):
        """ 좌측과 우측 차선의 곡률(반경)을 평균하여,
            차량 wheelbase를 고려한 조향각(steering angle, 단위: 도)을 계산
            steering_angle = arctan(wheelbase / R)
            방향은 'Left Curve'이면 음수, 'Right Curve'이면 양수, 'Straight'면 0으로 처리.
        """
        if left_curv is not None and right_curv is not None:
            R = (left_curv + right_curv) / 2.0
        elif left_curv is not None:
            R = left_curv
        elif right_curv is not None:
            R = right_curv
        else:
            return 0.0
        angle_rad = np.arctan(self.wheelbase / R)
        angle_deg = np.degrees(angle_rad)
        if curve_info[1] == "Left Curve":
            angle_deg = -angle_deg
        elif curve_info[1] == "Straight":
            angle_deg = 0.0
        return angle_deg

    def filter_colors(self, img_frame):
        """ 차선 검출을 위한 색상 필터링 """
        hsv = cv2.cvtColor(img_frame, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 120])
        upper_white = np.array([255, 255, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        lower_yellow = np.array([15, 80, 120])
        upper_yellow = np.array([50, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.bitwise_or(mask_white, mask_yellow)
        result = cv2.bitwise_and(img_frame, img_frame, mask=mask)
        return result

    def limit_region(self, img_edges):
        """ 관심영역(ROI) 설정 (정규화 좌표를 픽셀 좌표로 변환) """
        height, width = img_edges.shape
        mask = np.zeros_like(img_edges)
        ul = (int(self.roi_upper_left[0] * width), int(self.roi_upper_left[1] * height))
        ur = (int(self.roi_upper_right[0] * width), int(self.roi_upper_right[1] * height))
        lr = (int(self.roi_lower_right[0] * width), int(self.roi_lower_right[1] * height))
        ll = (int(self.roi_lower_left[0] * width), int(self.roi_lower_left[1] * height))
        vertices = np.array([[ul, ur, lr, ll]], dtype=np.int32)
        cv2.fillPoly(mask, [vertices], 255)
        masked_edges = cv2.bitwise_and(img_edges, mask)
        return masked_edges

    def houghLines(self, img_mask):
        """ Hough 변환을 이용하여 차선 검출 """
        lines = cv2.HoughLinesP(img_mask, 1, np.pi/180, 30, minLineLength=80, maxLineGap=60)
        return lines if lines is not None else []

    def separateLine(self, img_edges, lines):
        """ 검출된 선을 좌우로 구분 (기울기 기준) """
        left_lines, right_lines = [], []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 0
            if slope < -0.3:
                left_lines.append(line)
            elif slope > 0.3:
                right_lines.append(line)
        return [left_lines, right_lines]

    def regression(self, separated_lines, img_input):
        """ 선형 회귀를 이용해 좌우 차선에 대해 대표 선(두 점)을 계산 """
        lane = []
        height, width, _ = img_input.shape
        for side in separated_lines:
            if len(side) == 0:
                lane.append(None)
                continue
            x_vals, y_vals = [], []
            for line in side:
                x1, y1, x2, y2 = line[0]
                x_vals += [x1, x2]
                y_vals += [y1, y2]
            poly = np.polyfit(y_vals, x_vals, 1)
            poly_fn = np.poly1d(poly)
            y1, y2 = height, int(height * 0.6)
            x1, x2 = int(poly_fn(y1)), int(poly_fn(y2))
            lane.append((x1, y1, x2, y2))
        return lane

    def computeVanishingPoint(self, left_line, right_line):
        """ 좌측, 우측 회귀선(두 점) 연장을 통해 소실점(교차점)을 계산 """
        if left_line is None or right_line is None:
            return None
        x1, y1, x2, y2 = left_line
        x3, y3, x4, y4 = right_line
        if (x2 - x1) == 0 or (x4 - x3) == 0:
            return None
        m1 = (y2 - y1) / (x2 - x1)
        m2 = (y4 - y3) / (x4 - x3)
        if m1 == m2:
            return None
        b1 = y1 - m1 * x1
        b2 = y3 - m2 * x3
        vx = (b2 - b1) / (m1 - m2)
        vy = m1 * vx + b1
        return (int(vx), int(vy))

    def calculateCurvePixel(self, vanishing_point, img_input):
        """ 소실점과 기준점(0.55, 0.95) 사이의 수평 픽셀 차이로 간단 곡률(픽셀 기반)을 판단 """
        height, width, _ = img_input.shape
        ref_point = (int(width * self.ref_norm[0]), int(height * self.ref_norm[1]))
        dx = vanishing_point[0] - ref_point[0]
        if abs(dx) <= self.pixel_threshold:
            return dx, "Straight"
        elif dx < -self.pixel_threshold:
            return dx, "Left Curve"
        else:
            return dx, "Right Curve"

    def drawVisualization(self, img_input, left_line, right_line, vanishing_point,
                          curve_info, left_curv, right_curv, steering_angle):
        """ 별도 시각화 창:
            1) 소실점까지 연장된 좌측(초록) & 우측(파란) 선
            2) 소실점 (빨간 점)
            3) 노란색 박스: 차량 중앙 기준 좌우 ±20픽셀, 소실점 기준 상하 ±10픽셀
            4) 박스 중앙과 소실점을 잇는 빨간 얇은 선
            5) 픽셀 기반 곡률 및 차량 조향각(steering angle, 단위: 도) 텍스트 표시
        """
        vis_img = img_input.copy()
        height, width, _ = img_input.shape
        ref_point = (int(width * self.ref_norm[0]), int(height * self.ref_norm[1]))
        
        # 좌측 선 (초록색)
        if left_line is not None:
            lx, ly, _, _ = left_line
            cv2.line(vis_img, (lx, ly), vanishing_point, (0, 255, 0), 5)
        # 우측 선 (파란색)
        if right_line is not None:
            rx, ry, _, _ = right_line
            cv2.line(vis_img, (rx, ry), vanishing_point, (255, 0, 0), 5)
        # 소실점 (빨간색 점)
        cv2.circle(vis_img, vanishing_point, 5, (0, 0, 255), -1)
        # 노란색 박스: 차량 중앙 기준 좌우 ±20픽셀, 소실점 기준 상하 ±10픽셀
        box_tl = (ref_point[0] - self.box_horiz_tolerance, vanishing_point[1] - self.box_vert_tolerance)
        box_br = (ref_point[0] + self.box_horiz_tolerance, vanishing_point[1] + self.box_vert_tolerance)
        cv2.rectangle(vis_img, box_tl, box_br, (0, 255, 255), 2)
        # 박스 중앙과 소실점을 잇는 빨간색 얇은 선
        box_center = (ref_point[0], vanishing_point[1])
        cv2.line(vis_img, box_center, vanishing_point, (0, 0, 255), 1)
        # 기준점 (흰색 점)
        cv2.circle(vis_img, ref_point, 5, (255, 255, 255), -1)
        
        # 텍스트: 픽셀 기반 곡률 및 차량 조향각 (단위: 도)
        text1 = f"{curve_info[1]} (dx: {curve_info[0]:.2f} px)"
        cv2.putText(vis_img, text1, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
        steering_text = f"Steering Angle: {steering_angle:.2f} deg"
        cv2.putText(vis_img, steering_text, (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return vis_img

    def drawLine(self, img_input, lane):
        """ 원본 영상 위에 회귀 선(좌측, 우측)을 그리기 """
        img_output = img_input.copy()
        for line in lane:
            if line is not None:
                x1, y1, x2, y2 = line
                cv2.line(img_output, (x1, y1), (x2, y2), (0, 255, 0), 5)
        return img_output

    def image_callback(self, ros_image):
        """ ROS Subscriber 콜백: 이미지 수신 → 차선 검출 → 시각화 및 Publish """
        try:
            # ROS Image → OpenCV 이미지 변환
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            # 이미지 왜곡 보정
            if self.dist is not None:
                cv_image = cv2.undistort(cv_image, self.K, self.dist)
            # 1. 색상 필터링
            filtered_frame = self.filter_colors(cv_image)
            # 2. 그레이스케일 변환 및 CLAHE 적용
            gray_frame = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(16, 16))
            gray_frame = clahe.apply(gray_frame)
            # 3. Canny Edge Detection 적용
            edges = cv2.Canny(gray_frame, 65, 150, L2gradient=True)
            # 4. 관심영역(ROI) 마스킹
            roi_edges = self.limit_region(edges)
            # 5. Hough 변환으로 선 검출 후 좌우 분리
            lines = self.houghLines(roi_edges)
            if lines is not None and len(lines) > 0:
                separated_lines = self.separateLine(roi_edges, lines)
                lane = self.regression(separated_lines, cv_image)
                result_frame = self.drawLine(cv_image, lane)
                
                # 소실점 계산 (좌측, 우측 회귀선 모두 존재하면)
                if lane[0] is not None and lane[1] is not None:
                    vanishing_point = self.computeVanishingPoint(lane[0], lane[1])
                    if vanishing_point is not None:
                        cv2.circle(result_frame, vanishing_point, 5, (0, 0, 255), -1)
                        # 간단 픽셀 차이 기반 곡률 판단
                        curve_info = self.calculateCurvePixel(vanishing_point, cv_image)
                        
                        # 각 차선의 실제 곡률 계산 (여기서는 개별 곡률을 구한 후 평균으로 사용)
                        left_points = self.gatherLanePoints(separated_lines[0])
                        right_points = self.gatherLanePoints(separated_lines[1])
                        left_curv, right_curv = None, None
                        if len(left_points) >= 3:
                            left_curv, _ = self.computeLaneCurvature(left_points)
                        if len(right_points) >= 3:
                            right_curv, _ = self.computeLaneCurvature(right_points)
                        
                        # 조향각(steering angle) 계산: 아크탄젠트를 이용 (wheelbase / R)
                        steering_angle = self.computeSteeringAngle(left_curv, right_curv, curve_info)
                        
                        # 결과 영상에 픽셀 기반 곡률 텍스트 오버레이
                        text = f"{curve_info[1]} (dx: {curve_info[0]:.2f} px)"
                        cv2.putText(result_frame, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        
                        # 별도 시각화 창 생성: 좌측/우측 선, 소실점, 노란 박스, 빨간 선, 조향각 텍스트 표시
                        vis_img = self.drawVisualization(cv_image, lane[0], lane[1], vanishing_point,
                                                        curve_info, left_curv, right_curv, steering_angle)
                        cv2.imshow("Lane Visualization", vis_img)
            else:
                result_frame = cv_image.copy()

            cv2.imshow("ROI", roi_edges)
            cv2.imshow("Lane Detection", result_frame)
            cv2.waitKey(1)

            ros_output = self.bridge.cv2_to_imgmsg(result_frame, "bgr8")
            self.image_pub.publish(ros_output)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)

def main():
    rospy.init_node("lane_detection_node", anonymous=True)
    detector = RoadLaneDetector()
    rospy.spin()

if __name__ == "__main__":
    main()

