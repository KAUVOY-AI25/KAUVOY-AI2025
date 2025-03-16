import rospy
import cv2
import numpy as np
import pickle
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class FixedHomographyStitcher:
    def __init__(self):
        rospy.init_node('fixed_homography_stitcher', anonymous=True)

        self.left_sub = rospy.Subscriber('/camera1/usb_cam1/image_raw', Image, self.left_image_callback)
        self.right_sub = rospy.Subscriber('/camera2/usb_cam2/image_raw', Image, self.right_image_callback)

        self.panorama_pub = rospy.Publisher('/panorama/image', Image, queue_size=1)

        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None

        self.best_H = self.load_homography_matrix('/home/kauvoy/ch/best_homography.pkl')

    def load_homography_matrix(self, path):
        try:
            with open(path, 'rb') as f:
                H = pickle.load(f)
            rospy.loginfo("Homography matrix loaded successfully.")
            return H
        except Exception as e:
            rospy.logerr(f"Error loading homography matrix: {e}")
            return None

    def left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def process_images(self):
        if self.left_image is None or self.right_image is None:
            rospy.logwarn("Waiting for images from both cameras.")
            return

        if self.best_H is None:
            rospy.logwarn("No homography matrix available.")
            return

        panorama = self.make_panorama(self.left_image, self.right_image)
        if panorama is not None:
            self.publish_panorama(panorama)

    def make_panorama(self, left_image, right_image):
        try:
            height = max(left_image.shape[0], right_image.shape[0])
            width = left_image.shape[1] + right_image.shape[1]
            result = cv2.warpPerspective(right_image, self.best_H, (width, height))
            result[0:left_image.shape[0], 0:left_image.shape[1]] = left_image
            return result
        except Exception as e:
            rospy.logerr(f"Error during panorama stitching: {e}")
            return None

    def publish_panorama(self, panorama):
        try:
            panorama_msg = self.bridge.cv2_to_imgmsg(panorama, 'bgr8')
            self.panorama_pub.publish(panorama_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing panorama image: {e}")

if __name__ == '__main__':
    node = FixedHomographyStitcher()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.process_images()
        rate.sleep()

