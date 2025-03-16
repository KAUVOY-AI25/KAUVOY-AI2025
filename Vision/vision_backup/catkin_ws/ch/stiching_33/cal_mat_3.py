import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class HomographySaver:
    def __init__(self):
        rospy.init_node('homography_saver', anonymous=True)

        # 카메라 이미지 구독
        self.left_sub = rospy.Subscriber('/camera1/usb_cam1/image_raw', Image, self.left_image_callback)
        self.center_sub = rospy.Subscriber('/camera3/usb_cam3/image_raw', Image, self.center_image_callback)
        self.right_sub = rospy.Subscriber('/camera2/usb_cam2/image_raw', Image, self.right_image_callback)

        self.bridge = CvBridge()
        self.left_image = None
        self.center_image = None
        self.right_image = None

        self.best_H1 = None  # 왼쪽 → 중앙
        self.best_H2 = None  # 오른쪽 → 중앙

        self.calibration_time = rospy.get_param('~calibration_time', 30)
        self.start_time = rospy.Time.now()

        # 호모그래피 행렬 저장 경로
        self.save_path = "/home/kauvoy/ch/stiching_3"
        os.makedirs(self.save_path, exist_ok=True)  # 폴더 없으면 생성

        self.h1_path = os.path.join(self.save_path, "best_homography_left.npy")
        self.h2_path = os.path.join(self.save_path, "best_homography_right.npy")

    def left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def center_image_callback(self, msg):
        self.center_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def process_images(self):
        if self.left_image is None or self.center_image is None or self.right_image is None:
            rospy.logwarn("Waiting for images from all three cameras.")
            return

        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.start_time).to_sec()

        if elapsed_time < self.calibration_time:
            self.best_H1 = self.calibrate_homography(self.left_image, self.center_image, "Left-Center")
            self.best_H2 = self.calibrate_homography(self.right_image, self.center_image, "Right-Center")
        else:
            if self.best_H1 is not None and self.best_H2 is not None:
                np.save(self.h1_path, self.best_H1)
                np.save(self.h2_path, self.best_H2)
                rospy.loginfo(f"Homography matrices saved in {self.save_path}")
                rospy.signal_shutdown("Calibration completed.")

    def calibrate_homography(self, src_image, dst_image, label):
        """ 중앙 카메라 기준으로 src_image를 dst_image에 맞춰는 호모그래피를 계산 """
        try:
            gray_src = cv2.cvtColor(src_image, cv2.COLOR_BGR2GRAY)
            gray_dst = cv2.cvtColor(dst_image, cv2.COLOR_BGR2GRAY)

            akaze = cv2.AKAZE_create()
            keypoints_src, descriptors_src = akaze.detectAndCompute(gray_src, None)
            keypoints_dst, descriptors_dst = akaze.detectAndCompute(gray_dst, None)

            matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
            knn_matches = matcher.knnMatch(descriptors_src, descriptors_dst, k=2)

            good_matches = [m for m, n in knn_matches if m.distance < 0.75 * n.distance]

            rospy.loginfo(f"Number of Good Matches ({label}): {len(good_matches)}")
            if len(good_matches) < 10:
                rospy.logwarn(f"Not enough good matches for {label}.")
                return None

            points_src = np.float32([keypoints_src[m.queryIdx].pt for m in good_matches])
            points_dst = np.float32([keypoints_dst[m.trainIdx].pt for m in good_matches])

            if len(points_src) < 4 or len(points_dst) < 4:
                rospy.logwarn(f"Not enough points for {label} homography.")
                return None

            # 호모그래피 변환 적용 (중앙을 기준으로 좌표 이동)
            H, _ = cv2.findHomography(points_src, points_dst, cv2.RANSAC, 5.0)
            rospy.loginfo(f"Calibrated Homography Matrix ({label}): \n{H}")

            # 실시간 학습점 메\uuce6 매칭 시간화
            match_image = self.draw_matches(src_image, dst_image, keypoints_src, keypoints_dst, good_matches)
            cv2.imshow(f"Feature Matching {label}", match_image)
            cv2.waitKey(1)

            return H

        except Exception as e:
            rospy.logerr(f"Error during {label} homography calibration: {e}")
            return None

    def draw_matches(self, img1, img2, keypoints1, keypoints2, matches):
        """ 학습점 매칭을 시간시 시각화하는 함수 """
        match_img = cv2.drawMatches(img1, keypoints1, img2, keypoints2, matches, None,
                                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        return match_img

if __name__ == '__main__':
    node = HomographySaver()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.process_images()
        rate.sleep()
    cv2.destroyAllWindows()  # 종료 시 창 닫기

