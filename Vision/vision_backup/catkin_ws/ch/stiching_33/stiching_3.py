import rospy
import cv2
import numpy as np
import pickle
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PanoramaStitcher:
    def __init__(self):
        rospy.init_node('panorama_stitcher', anonymous=True)

        # 카메라 이미지 구독
        self.left_sub = rospy.Subscriber('/camera1/usb_cam1/image_raw', Image, self.left_image_callback)
        self.center_sub = rospy.Subscriber('/camera3/usb_cam3/image_raw', Image, self.center_image_callback)
        self.right_sub = rospy.Subscriber('/camera2/usb_cam2/image_raw', Image, self.right_image_callback)

        # 파노라마 퍼블리셔
        self.panorama_pub = rospy.Publisher('/panorama/image', Image, queue_size=1)

        self.bridge = CvBridge()
        self.left_image = None
        self.center_image = None
        self.right_image = None

        # 호모그래피 행렬 저장 경로
        self.homography_path = "/home/kauvoy/ch/stiching_3"

        self.H1 = self.load_homography_matrix(os.path.join(self.homography_path, "best_homography_left.npy"))  # 왼쪽 → 중앙
        self.H2 = self.load_homography_matrix(os.path.join(self.homography_path, "best_homography_right.npy"))  # 오른쪽 → 중앙

    def load_homography_matrix(self, path):
        """ 저장된 호모그래피 행렬을 로드하는 함수 """
        try:
            with open(path, 'rb') as f:
                H = pickle.load(f)
            rospy.loginfo(f"Loaded homography matrix from {path}")
            return H
        except Exception as e:
            rospy.logerr(f"Error loading homography matrix from {path}: {e}")
            return None

    def left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def center_image_callback(self, msg):
        self.center_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def process_images(self):
        """ stitching 수행 후 ROS 퍼블리시 """
        if self.left_image is None or self.center_image is None or self.right_image is None:
            rospy.logwarn("Waiting for all camera images.")
            return

        if self.H1 is None or self.H2 is None:
            rospy.logwarn("Homography matrices not loaded.")
            return

        # stitching 수행
        panorama = self.make_panorama(self.left_image, self.center_image, self.right_image)
        if panorama is not None:
            self.publish_panorama(panorama)

    def make_panorama(self, left_image, center_image, right_image):
        """ 중앙 카메라 기준으로 왼쪽, 오른쪽 stitching 수행 (중앙 고정) """
        try:
            height, width = center_image.shape[:2]
            output_width = width * 3  # 중앙 기준으로 좌우 확장

            # 중앙 카메라 좌표계를 유지하기 위한 변환 행렬 (중앙 고정)
            identity_matrix = np.array([[1, 0, width], [0, 1, 0], [0, 0, 1]], dtype=np.float32)

            # 왼쪽 이미지를 중앙 기준으로 변환 (왼쪽 이동)
            left_warped = cv2.warpPerspective(left_image, self.H1 @ identity_matrix, (output_width, height))

            # 오른쪽 이미지를 중앙 기준으로 변환 (오른쪽 이동)
            right_warped = cv2.warpPerspective(right_image, self.H2 @ np.array([[1, 0, width * 2], [0, 1, 0], [0, 0, 1]], dtype=np.float32), (output_width, height))

            # 중앙 이미지를 원래 위치에 배치
            panorama = np.zeros((height, output_width, 3), dtype=np.uint8)
            panorama[:, width:width*2] = center_image  # 중앙 카메라 고정

            # 왼쪽과 오른쪽 이미지 합성
            mask = (left_warped > 0)
            panorama[mask] = left_warped[mask]
            mask = (right_warped > 0)
            panorama[mask] = right_warped[mask]

            return panorama
        except Exception as e:
            rospy.logerr(f"Error during panorama stitching: {e}")
            return None

    def publish_panorama(self, panorama):
        """ ROS 토픽으로 stitching된 이미지 퍼블리시 """
        try:
            panorama_msg = self.bridge.cv2_to_imgmsg(panorama, 'bgr8')
            self.panorama_pub.publish(panorama_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing panorama image: {e}")

if __name__ == '__main__':
    node = PanoramaStitcher()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.process_images()
        rate.sleep()

