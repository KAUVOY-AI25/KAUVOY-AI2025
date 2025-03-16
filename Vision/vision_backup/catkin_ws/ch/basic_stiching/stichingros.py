# ================================================================================
# Main Author: 이창현, 윤수빈
# Recently Modified Date: 2024-12-22 (V1.0)
# Dependency: multicam
# Description: 웹캠 2대를 스티칭 기법을 사용하여 파노라마 뷰로 제작. (30초간 호모그래피 최적화 후 고정)
# ================================================================================

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PanoramaNode:
    def __init__(self):
        rospy.init_node('panorama_node', anonymous=True)

        self.left_sub = rospy.Subscriber('/camera1/usb_cam1/image_raw', Image, self.left_image_callback)  #왼쪽 카메라 확인 후 수정
        self.right_sub = rospy.Subscriber('/camera2/usb_cam2/image_raw', Image, self.right_image_callback) #오른쪽 카메라 확인 후 수정

        self.panorama_pub = rospy.Publisher('/panorama/image', Image, queue_size=1) #publish 할 토픽

        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.best_H = None  #최적 호모그래피 행렬 저장
        self.calibration_time = rospy.get_param('~calibration_time', 15)  # 캘리브레이션 진행 할 시간 : 이 이후 최적 값으로 화면 고정됨
        self.start_time = rospy.Time.now()

    def left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  #왼쪽 카메라 이미지 콜백 : ros 이미지를 cv 이미지로 변환

    def right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  #오른쪽 카메라 이미지 콜백 : ros 이미지를 cv 이미지로 변환

    def process_images(self):
        if self.left_image is None or self.right_image is None:
            rospy.logwarn("Waiting for images from both cameras.")  #예외처리 : 두 카메라에서 이미지 받았는 지 확인
            return

        current_time = rospy.Time.now()  #현재 시간 받아오기
        #캘리 시간 이내에서 호모그래피 최적화 수행
        if (current_time - self.start_time).to_sec() < self.calibration_time:
            self.best_H = self.calibrate_homography(self.left_image, self.right_image)
        else:
        #최적화 완료 후에는 저장된 호모그래피 행렬 사용 파노라마 생성
            if self.best_H is not None:
                panorama = self.make_panorama_with_fixed_H(self.left_image, self.right_image)
                if panorama is not None:
                    self.publish_panorama(panorama) #퍼블리시

    def calibrate_homography(self, left_image, right_image):
        #호모그래피 행렬 계산
        try:
            gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY) #왼쪽 이미지를 그레이 스케일로 변환
            gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY) #오른쪽 이미지를 그레이 스케일로 변환

            # AKAZE 특징점 검출 및 디스크립터 계산 : 알고리즘 변경 가능함
            akaze = cv2.AKAZE_create()
            keypoints_left, descriptors_left = akaze.detectAndCompute(gray_left, None)
            keypoints_right, descriptors_right = akaze.detectAndCompute(gray_right, None)

            # KNN 매칭을 사용한 두 이미지 특징점 매칭
            matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
            #k는 특징점 쌍의 개수 : k 증가 시 많은 후보 증가로 정확도 높아지지만 계산량 증가, 감소 시 빠른 계산, 낮은 정확도
            knn_matches = matcher.knnMatch(descriptors_left, descriptors_right, k=2)

            # Lowe's ration test를 적용하여 좋은 매칭점 필터링
            good_matches = []
            for m, n in knn_matches:
                 #lowe's ration 증가 시 더 많은 매칭 통과 but 잘못된 매칭 가능 / 감소시 매칭 품질 향상 but 매칭 잃을 가능성
                if m.distance < 0.75 * n.distance: 
                    good_matches.append(m)

            rospy.loginfo(f"Number of Good Matches (Calibration): {len(good_matches)}")  #매칭된 특징점 개수 출력
            if len(good_matches) < 10:
                rospy.logwarn("Not enough good matches for homography calibration.")  #매칭점 부족할 때 뜨는 경고문 -> 카메라 각도 조절 필요
                return self.best_H

            #매칭된 특징점 좌표를 추출함
            points_left = np.float32([keypoints_left[m.queryIdx].pt for m in good_matches])
            points_right = np.float32([keypoints_right[m.trainIdx].pt for m in good_matches])

            if len(points_left) < 4 or len(points_right) < 4:  #호모그래피 행렬이 부족함
                rospy.logwarn("Not enough points for homography calibration.")
                return self.best_H

            #계산된 호모그래피 행렬을 출력
            #ransac 계수 변경 가능 : 높이면 많은 점 포함 but 왜곡 가능성 / 낮추면 품질 향상 but 유효점 배제 가능
            H, _ = cv2.findHomography(points_right, points_left, cv2.RANSAC, 5.0)
            rospy.loginfo(f"Calibrated Homography Matrix: \n{H}")
            return H
        #호모그래피 행렬의 에러가 발생하였을 경우
        except Exception as e:
            rospy.logerr(f"Error during homography calibration: {e}")
            return self.best_H

    def make_panorama_with_fixed_H(self, left_image, right_image):
        #파노라마 생성하는 부분
        try:
            if self.best_H is None:
                rospy.logwarn("Homography matrix is not calibrated.") #호모그래피 행렬이 없을 때 경고
                return None

            height = max(left_image.shape[0], right_image.shape[0]) #파노라마 이미지 높이 계산
            width = left_image.shape[1] + right_image.shape[1] #파노라마 이미지 넓이 계산
            result = cv2.warpPerspective(right_image, self.best_H, (width, height)) #오른쪽 이미지를 변환
            result[0:left_image.shape[0], 0:left_image.shape[1]] = left_image #왼쪽 이미지를 결과 이미지에 복사
            return result
        except Exception as e:
            rospy.logerr(f"Error during panorama stitching with fixed H: {e}")
            return None

    def publish_panorama(self, panorama):
        try:
            panorama_msg = self.bridge.cv2_to_imgmsg(panorama, 'bgr8') #cv 이미지를 ros 메시지로 변환
            self.panorama_pub.publish(panorama_msg) #퍼블리시
        except Exception as e:
            rospy.logerr(f"Error publishing panorama image: {e}")

if __name__ == '__main__':
    node = PanoramaNode()
    rate = rospy.Rate(10) #루프 주기 설정 (10Hz)
    while not rospy.is_shutdown():
        node.process_images() #이미지 처리 수행
        rate.sleep() #루프 대기
