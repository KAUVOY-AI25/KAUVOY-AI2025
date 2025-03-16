#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PanoramaStitcher:
    def __init__(self):
        rospy.init_node('panorama_publisher', anonymous=True)
        self.image_pub = rospy.Publisher('param', Image, queue_size=1)
        self.bridge = CvBridge()
        
        # 이미지 저장용 변수
        self.left_image = None
        self.center_image = None
        self.right_image = None

        # ROS Subscriber 설정
        rospy.Subscriber("/camera1/usb_cam1/image_raw", Image, self.left_callback)
        rospy.Subscriber("/camera3/usb_cam3/image_raw", Image, self.center_callback)
        rospy.Subscriber("/camera2/usb_cam2/image_raw", Image, self.right_callback)

        # 호모그래피 행렬 로드
        self.H_left_center = self.load_homography("/home/kauvoy/ch/matrix/homography_left_center.npy")
        self.H_center_right = self.load_homography("/home/kauvoy/ch/matrix/homography_center_right.npy")

    def load_homography(self, filename):
        try:
            return np.load(filename).astype(np.float32)
        except FileNotFoundError:
            rospy.logerr(f"Homography matrix file {filename} not found.")
            return None

    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def center_callback(self, msg):
        self.center_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def stitch_panorama(self):
        if self.H_left_center is None or self.H_center_right is None:
            return

        if self.left_image is None or self.center_image is None or self.right_image is None:
            rospy.logwarn("Waiting for all camera images...")
            return

        left_image_flipped = cv2.flip(self.left_image, 1)
        center_image_flipped = cv2.flip(self.center_image, 1)
        
        height_left_center = max(left_image_flipped.shape[0], center_image_flipped.shape[0])
        width_left_center = left_image_flipped.shape[1] + center_image_flipped.shape[1]

        stitched_left_center = cv2.warpPerspective(left_image_flipped, self.H_left_center, (width_left_center, height_left_center), flags=cv2.INTER_LINEAR)
        stitched_left_center[:center_image_flipped.shape[0], :center_image_flipped.shape[1]] = center_image_flipped

        height_center_right = max(self.center_image.shape[0], self.right_image.shape[0])
        width_center_right = self.center_image.shape[1] + self.right_image.shape[1]
        stitched_center_right = cv2.warpPerspective(self.right_image, self.H_center_right, (width_center_right, height_center_right), flags=cv2.INTER_LINEAR)
        stitched_center_right[:self.center_image.shape[0], :self.center_image.shape[1]] = self.center_image

        height_final = max(stitched_left_center.shape[0], stitched_center_right.shape[0])
        width_final = stitched_left_center.shape[1] + stitched_center_right.shape[1] - self.center_image.shape[1]

        stitched_final = np.zeros((height_final, width_final, 3), dtype=np.uint8)
        stitched_final[:stitched_left_center.shape[0], :stitched_left_center.shape[1]] = stitched_left_center
        stitched_final[:stitched_center_right.shape[0], stitched_left_center.shape[1]:] = stitched_center_right[:, self.center_image.shape[1]:]

        stitched_final[:, :stitched_left_center.shape[1]] = cv2.flip(stitched_final[:, :stitched_left_center.shape[1]], 1)

        cv2.imshow("Optimized Panorama", stitched_final)
        cv2.waitKey(10)
        
        # ROS 토픽으로 퍼블리시
        ros_image = self.bridge.cv2_to_imgmsg(stitched_final, encoding="bgr8")
        self.image_pub.publish(ros_image)

if __name__ == "__main__":
    stitcher = PanoramaStitcher()
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        stitcher.stitch_panorama()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rate.sleep()
    cv2.destroyAllWindows()
