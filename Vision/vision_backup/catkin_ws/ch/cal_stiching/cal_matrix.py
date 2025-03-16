import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pickle

class HomographySaver:
    def __init__(self):
        rospy.init_node('homography_saver', anonymous=True)

        self.left_sub = rospy.Subscriber('/camera1/usb_cam1/image_raw', Image, self.left_image_callback)
        self.right_sub = rospy.Subscriber('/camera2/usb_cam2/image_raw', Image, self.right_image_callback)

        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.best_H = None  
        self.calibration_time = rospy.get_param('~calibration_time', 30)  
        self.start_time = rospy.Time.now()

    def left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def process_images(self):
        if self.left_image is None or self.right_image is None:
            rospy.logwarn("Waiting for images from both cameras.")
            return

        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.start_time).to_sec()

        if elapsed_time < self.calibration_time:
            self.best_H = self.calibrate_homography(self.left_image, self.right_image)
        else:
            if self.best_H is not None:
                with open('/home/kauvoy/ch/best_homography.pkl', 'wb') as f:
                    pickle.dump(self.best_H, f)
                rospy.loginfo("Best homography matrix saved.")
                rospy.signal_shutdown("Calibration completed.")

    def calibrate_homography(self, left_image, right_image):
        try:
            gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

            akaze = cv2.AKAZE_create()
            keypoints_left, descriptors_left = akaze.detectAndCompute(gray_left, None)
            keypoints_right, descriptors_right = akaze.detectAndCompute(gray_right, None)

            matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
            knn_matches = matcher.knnMatch(descriptors_left, descriptors_right, k=2)

            good_matches = [m for m, n in knn_matches if m.distance < 0.75 * n.distance]

            rospy.loginfo(f"Number of Good Matches: {len(good_matches)}")
            if len(good_matches) < 10:
                rospy.logwarn("Not enough good matches.")
                return self.best_H

            points_left = np.float32([keypoints_left[m.queryIdx].pt for m in good_matches])
            points_right = np.float32([keypoints_right[m.trainIdx].pt for m in good_matches])

            if len(points_left) < 4 or len(points_right) < 4:
                rospy.logwarn("Not enough points for homography.")
                return self.best_H

            H, _ = cv2.findHomography(points_right, points_left, cv2.RANSAC, 5.0)
            rospy.loginfo(f"Calibrated Homography Matrix: \n{H}")
            return H

        except Exception as e:
            rospy.logerr(f"Error during homography calibration: {e}")
            return self.best_H

if __name__ == '__main__':
    node = HomographySaver()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.process_images()
        rate.sleep()

