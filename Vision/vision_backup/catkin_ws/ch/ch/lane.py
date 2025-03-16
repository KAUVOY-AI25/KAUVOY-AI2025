# # # import cv2
# # # import numpy as np

# # # class RoadLaneDetector:
# # #     def __init__(self):
# # #         self.img_size = 0
# # #         self.img_center = 0
# # #         self.left_m = 0
# # #         self.right_m = 0
# # #         self.left_b = None
# # #         self.right_b = None
# # #         self.left_detect = False
# # #         self.right_detect = False

# # #         # ROI 설정을 위한 파라미터
# # #         self.poly_bottom_width = 0.85  # 하단 너비 비율
# # #         self.poly_top_width = 0.07     # 상단 너비 비율
# # #         self.poly_height = 0.4         # ROI 높이 비율

# # #     def filter_colors(self, img_frame):
# # #         """ 차선 검출을 위한 색상 필터링 """
# # #         hsv = cv2.cvtColor(img_frame, cv2.COLOR_BGR2HSV)

# # #         # 흰색 차선 필터링
# # #         lower_white = np.array([0, 0, 200])
# # #         upper_white = np.array([255, 30, 255])
# # #         mask_white = cv2.inRange(hsv, lower_white, upper_white)

# # #         # 노란색 차선 필터링
# # #         lower_yellow = np.array([18, 94, 140])
# # #         upper_yellow = np.array([48, 255, 255])
# # #         mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

# # #         # 두 마스크 결합
# # #         mask = cv2.bitwise_or(mask_white, mask_yellow)
# # #         result = cv2.bitwise_and(img_frame, img_frame, mask=mask)

# # #         return result

# # #     def limit_region(self, img_edges):
# # #         """ 관심영역(ROI) 설정 """
# # #         height, width = img_edges.shape

# # #         # ROI 다각형 좌표 설정
# # #         bottom_left = (int(width * (1 - self.poly_bottom_width) / 2), height)
# # #         bottom_right = (int(width * (1 + self.poly_bottom_width) / 2), height)
# # #         top_left = (int(width * (1 - self.poly_top_width) / 2), int(height * (1 - self.poly_height)))
# # #         top_right = (int(width * (1 + self.poly_top_width) / 2), int(height * (1 - self.poly_height)))

# # #         vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)

# # #         mask = np.zeros_like(img_edges)
# # #         cv2.fillPoly(mask, vertices, 255)
# # #         masked_edges = cv2.bitwise_and(img_edges, mask)

# # #         return masked_edges

# # #     def houghLines(self, img_mask):
# # #         """ Hough 변환을 이용하여 차선 검출 """
# # #         lines = cv2.HoughLinesP(img_mask, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=50)
# # #         return lines if lines is not None else []

# # #     def separateLine(self, img_edges, lines):
# # #         """ 검출된 선을 좌우로 구분 """
# # #         left_lines = []
# # #         right_lines = []
# # #         height, width = img_edges.shape

# # #         for line in lines:
# # #             x1, y1, x2, y2 = line[0]
# # #             slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0  # 기울기 계산

# # #             if slope < -0.3:  # 좌측 차선
# # #                 left_lines.append(line)
# # #             elif slope > 0.3:  # 우측 차선
# # #                 right_lines.append(line)

# # #         return [left_lines, right_lines]

# # #     def regression(self, separated_lines, img_input):
# # #         """ 차선 선형 회귀 적용 """
# # #         lane = []
# # #         height, width, _ = img_input.shape

# # #         for side in separated_lines:
# # #             if len(side) == 0:
# # #                 lane.append(None)
# # #                 continue

# # #             x_vals = []
# # #             y_vals = []

# # #             for line in side:
# # #                 x1, y1, x2, y2 = line[0]
# # #                 x_vals += [x1, x2]
# # #                 y_vals += [y1, y2]

# # #             poly = np.polyfit(y_vals, x_vals, 1)  # 1차 다항 회귀 적용 (y 기준 x 예측)
# # #             poly_fn = np.poly1d(poly)

# # #             y1, y2 = height, int(height * (1 - self.poly_height))
# # #             x1, x2 = int(poly_fn(y1)), int(poly_fn(y2))

# # #             lane.append((x1, y1, x2, y2))

# # #         return lane


# # #     def drawLine(self, img_input, lane):
# # #         """ 차선을 영상에 그리기 """
# # #         img_output = img_input.copy()

# # #         for line in lane:
# # #             if line is not None:
# # #                 x1, y1, x2, y2 = line
# # #                 cv2.line(img_output, (x1, y1), (x2, y2), (0, 255, 0), 5)

# # #         return img_output

# # # def main():
# # #     road_lane_detector = RoadLaneDetector()
    
# # #     video = cv2.VideoCapture("input.mp4")
# # #     if not video.isOpened():
# # #         print("비디오를 열 수 없습니다.")
# # #         return -1

# # #     ret, img_frame = video.read()
# # #     if not ret:
# # #         return -1

# # #     fourcc = cv2.VideoWriter_fourcc(*'XVID')
# # #     fps = 25.0
# # #     filename = "./result.avi"
# # #     writer = cv2.VideoWriter(filename, fourcc, fps, (img_frame.shape[1], img_frame.shape[0]))

# # #     if not writer.isOpened():
# # #         print("비디오 저장을 열 수 없습니다.")
# # #         return -1

# # #     cnt = 0

# # #     while True:
# # #         ret, img_frame = video.read()
# # #         if not ret:
# # #             break

# # #         img_filter = road_lane_detector.filter_colors(img_frame)

# # #         img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2GRAY)

# # #         img_edges = cv2.Canny(img_filter, 50, 150)

# # #         img_mask = road_lane_detector.limit_region(img_edges)

# # #         lines = road_lane_detector.houghLines(img_mask)

# # #         if lines is not None and len(lines) > 0:
# # #             separated_lines = road_lane_detector.separateLine(img_mask, lines)
# # #             lane = road_lane_detector.regression(separated_lines, img_frame)


# # #             img_result = road_lane_detector.drawLine(img_frame, lane)
# # #         else:
# # #             img_result = img_frame.copy()

# # #         writer.write(img_result)
# # #         if cnt == 15:
# # #             cv2.imwrite("img_result.jpg", img_result)
# # #         cnt += 1

# # #         cv2.imshow("result", img_result)

# # #     video.release()
# # #     writer.release()
# # #     cv2.destroyAllWindows()

# # # if __name__ == "__main__":
# # #     main()



# import cv2
# import numpy as np

# class RoadLaneDetector:
#     def __init__(self):
#         self.poly_bottom_width = 0.85  # 하단 너비 비율
#         self.poly_top_width = 0.07     # 상단 너비 비율
#         self.poly_height = 0.4         # ROI 높이 비율

#     def filter_colors(self, img_frame):
#         """ 차선 검출을 위한 색상 필터링 """
#         hsv = cv2.cvtColor(img_frame, cv2.COLOR_BGR2HSV)

#         # 흰색 차선 필터링
#         lower_white = np.array([0, 0, 200])
#         upper_white = np.array([255, 30, 255])
#         mask_white = cv2.inRange(hsv, lower_white, upper_white)

#         # 노란색 차선 필터링
#         lower_yellow = np.array([18, 94, 140])
#         upper_yellow = np.array([48, 255, 255])
#         mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

#         # 두 마스크 결합
#         mask = cv2.bitwise_or(mask_white, mask_yellow)
#         result = cv2.bitwise_and(img_frame, img_frame, mask=mask)

#         return result

#     def limit_region(self, img_edges):
#         """ 관심영역(ROI) 설정 """
#         height, width = img_edges.shape
#         mask = np.zeros_like(img_edges)

#         # ROI 다각형 좌표 설정
#         bottom_left = (int(width * (1 - self.poly_bottom_width) / 2), height)
#         bottom_right = (int(width * (1 + self.poly_bottom_width) / 2), height)
#         top_left = (int(width * (1 - self.poly_top_width) / 2), int(height * (1 - self.poly_height)))
#         top_right = (int(width * (1 + self.poly_top_width) / 2), int(height * (1 - self.poly_height)))

#         vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
#         cv2.fillPoly(mask, [vertices], 255)
#         masked_edges = cv2.bitwise_and(img_edges, mask)

#         return masked_edges

#     def houghLines(self, img_mask):
#         """ Hough 변환을 이용하여 차선 검출 """
#         lines = cv2.HoughLinesP(img_mask, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=50)
#         return lines if lines is not None else []

#     def separateLine(self, img_edges, lines):
#         """ 검출된 선을 좌우로 구분 """
#         left_lines, right_lines = [], []
#         height, width = img_edges.shape

#         for line in lines:
#             x1, y1, x2, y2 = line[0]
#             slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0  # 기울기 계산

#             if slope < -0.3:  # 좌측 차선
#                 left_lines.append(line)
#             elif slope > 0.3:  # 우측 차선
#                 right_lines.append(line)

#         return [left_lines, right_lines]

#     def regression(self, separated_lines, img_input):
#         """ 차선 선형 회귀 적용 """
#         lane = []
#         height, width, _ = img_input.shape

#         for side in separated_lines:
#             if len(side) == 0:
#                 lane.append(None)
#                 continue

#             x_vals, y_vals = [], []
#             for line in side:
#                 x1, y1, x2, y2 = line[0]
#                 x_vals += [x1, x2]
#                 y_vals += [y1, y2]

#             poly = np.polyfit(y_vals, x_vals, 1)  # 1차 다항 회귀 적용 (y 기준 x 예측)
#             poly_fn = np.poly1d(poly)

#             y1, y2 = height, int(height * (1 - self.poly_height))
#             x1, x2 = int(poly_fn(y1)), int(poly_fn(y2))

#             lane.append((x1, y1, x2, y2))

#         return lane

#     def drawLine(self, img_input, lane):
#         """ 차선을 영상에 그리기 """
#         img_output = img_input.copy()

#         # 차선 그리기
#         for line in lane:
#             if line is not None:
#                 x1, y1, x2, y2 = line
#                 cv2.line(img_output, (x1, y1), (x2, y2), (0, 255, 0), 5)

#         return img_output

# def main():
#     detector = RoadLaneDetector()

#     # 🟢 비디오 입력 선택 (웹캠: 0, 파일: "input.mp4")
#     video = cv2.VideoCapture("input.mp4")  # 웹캠 사용 시 → cv2.VideoCapture(0)
#     if not video.isOpened():
#         print("비디오를 열 수 없습니다.")
#         return

#     # 프레임 정보 가져오기
#     ret, frame = video.read()
#     if not ret:
#         print("비디오 프레임을 읽을 수 없습니다.")
#         return

#     height, width = frame.shape[:2]

#     # 🟢 MP4 파일로 저장 (코덱: H.264 / AVC)
#     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#     writer = cv2.VideoWriter("output.mp4", fourcc, 25, (width, height))

#     while True:
#         ret, frame = video.read()
#         if not ret:
#             break

#         # 1. 색상 필터링
#         filtered_frame = detector.filter_colors(frame)

#         # 2. 그레이스케일 변환
#         gray_frame = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)

#         # 3. Canny Edge Detection 적용
#         edges = cv2.Canny(gray_frame, 50, 150)

#         # 4. 관심 영역(ROI) 마스킹
#         roi_edges = detector.limit_region(edges)

#         # 5. Hough 변환을 사용한 직선 검출
#         lines = detector.houghLines(roi_edges)

#         # 🟢 ✅ 수정된 조건문: `if lines is not None and len(lines) > 0:`
#         if lines is not None and len(lines) > 0:
#             # 6. 검출된 선을 좌/우 구분
#             separated_lines = detector.separateLine(roi_edges, lines)

#             # 7. 선형 회귀 적용
#             lane = detector.regression(separated_lines, frame)

#             # 8. 차선 그리기
#             result_frame = detector.drawLine(frame, lane)
#         else:
#             result_frame = frame.copy()

#         # 🟢 실시간 화면 출력
#         cv2.imshow("Lane Detection", result_frame)

#         # 🟢 MP4 파일 저장
#         writer.write(result_frame)

#         # ESC 키 입력 시 종료
#         if cv2.waitKey(1) == 27:
#             break

#     video.release()
#     writer.release()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()


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
        self.image_sub = rospy.Subscriber("/camera2/usb_cam2/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/lane_raw", Image, queue_size=1)

        # ROI 파라미터 설정
        self.poly_bottom_width = 0.85  # 하단 너비 비율
        self.poly_top_width = 0.07     # 상단 너비 비율
        self.poly_height = 0.4         # ROI 높이 비율

    def filter_colors(self, img_frame):
        """ 차선 검출을 위한 색상 필터링 """
        hsv = cv2.cvtColor(img_frame, cv2.COLOR_BGR2HSV)

        # 흰색 차선 필터링
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([255, 30, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # 노란색 차선 필터링
        lower_yellow = np.array([18, 94, 140])
        upper_yellow = np.array([48, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 두 마스크 결합
        mask = cv2.bitwise_or(mask_white, mask_yellow)
        result = cv2.bitwise_and(img_frame, img_frame, mask=mask)

        return result

    def limit_region(self, img_edges):
        """ 관심영역(ROI) 설정 """
        height, width = img_edges.shape
        mask = np.zeros_like(img_edges)

        # ROI 다각형 좌표 설정
        bottom_left = (int(width * (1 - self.poly_bottom_width) / 2), height)
        bottom_right = (int(width * (1 + self.poly_bottom_width) / 2), height)
        top_left = (int(width * (1 - self.poly_top_width) / 2), int(height * (1 - self.poly_height)))
        top_right = (int(width * (1 + self.poly_top_width) / 2), int(height * (1 - self.poly_height)))

        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        cv2.fillPoly(mask, [vertices], 255)
        masked_edges = cv2.bitwise_and(img_edges, mask)

        return masked_edges

    def houghLines(self, img_mask):
        """ Hough 변환을 이용하여 차선 검출 """
        lines = cv2.HoughLinesP(img_mask, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=50)
        return lines if lines is not None else []

    def separateLine(self, img_edges, lines):
        """ 검출된 선을 좌우로 구분 """
        left_lines, right_lines = [], []
        height, width = img_edges.shape

        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0  # 기울기 계산

            if slope < -0.3:  # 좌측 차선
                left_lines.append(line)
            elif slope > 0.3:  # 우측 차선
                right_lines.append(line)

        return [left_lines, right_lines]

    def regression(self, separated_lines, img_input):
        """ 차선 선형 회귀 적용 """
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

            poly = np.polyfit(y_vals, x_vals, 1)  # 1차 다항 회귀 적용 (y 기준 x 예측)
            poly_fn = np.poly1d(poly)

            y1, y2 = height, int(height * (1 - self.poly_height))
            x1, x2 = int(poly_fn(y1)), int(poly_fn(y2))

            lane.append((x1, y1, x2, y2))

        return lane

    def drawLine(self, img_input, lane):
        """ 차선을 영상에 그리기 """
        img_output = img_input.copy()

        # 차선 그리기
        for line in lane:
            if line is not None:
                x1, y1, x2, y2 = line
                cv2.line(img_output, (x1, y1), (x2, y2), (0, 255, 0), 5)

        return img_output

    def image_callback(self, ros_image):
        """ ROS Subscriber 콜백 함수 """
        try:
            # ROS Image → OpenCV Image 변환
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

            # 1. 색상 필터링
            filtered_frame = self.filter_colors(cv_image)

            # 2. 그레이스케일 변환
            gray_frame = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)

            # 3. Canny Edge Detection 적용
            edges = cv2.Canny(gray_frame, 50, 150)

            # 4. 관심 영역(ROI) 마스킹
            roi_edges = self.limit_region(edges)

            # 5. Hough 변환을 사용한 직선 검출
            lines = self.houghLines(roi_edges)

            if lines is not None and len(lines) > 0:
                # 6. 검출된 선을 좌/우 구분
                separated_lines = self.separateLine(roi_edges, lines)

                # 7. 선형 회귀 적용
                lane = self.regression(separated_lines, cv_image)

                # 8. 차선 그리기
                result_frame = self.drawLine(cv_image, lane)
            else:
                result_frame = cv_image.copy()

            # OpenCV 창에 표시
            cv2.imshow("Lane Detection", result_frame)
            cv2.waitKey(1)

            # OpenCV Image → ROS Image 변환 후 Publish
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
