#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PanoramaStitcherROS:
    def __init__(self):
        rospy.init_node('panorama_stitcher_ros', anonymous=True)
        self.bridge = CvBridge()
        self.panorama_pub = rospy.Publisher('/panorama', Image, queue_size=1)

        # 구독 토픽: 오른쪽: /camera1, 중앙: /camera2, 왼쪽: /camera3
        self.right_sub = rospy.Subscriber('/camera1/usb_cam1/image_raw', Image, self.right_callback)
        self.center_sub = rospy.Subscriber('/camera2/usb_cam2/image_raw', Image, self.center_callback)
        self.left_sub = rospy.Subscriber('/camera3/usb_cam3/image_raw', Image, self.left_callback)

        # 이미지 저장 변수
        self.right_image = None
        self.center_image = None
        self.left_image = None

        # 미리 계산된 호모그래피 행렬 로드  
        # H_left_center는 왼쪽 이미지에 대해 (flip 없이) 계산되었고,
        # H_center_right는 오른쪽 이미지에 대해 계산되었으나, 실제 적용 시 오른쪽 이미지를 flip해야 함.
        self.H_left_center = np.load("homography_left_center.npy").astype(np.float32)
        self.H_center_right = np.load("homography_center_right.npy").astype(np.float32)

    def right_callback(self, msg):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Right image conversion failed: " + str(e))

    def center_callback(self, msg):
        try:
            self.center_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Center image conversion failed: " + str(e))

    def left_callback(self, msg):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Left image conversion failed: " + str(e))

    def horizontal_flip_matrix(self, width):
        """
        수평 flip 행렬: F = [ [-1, 0, width-1],
                                  [ 0, 1,       0],
                                  [ 0, 0,       1] ]
        """
        F = np.array([[-1, 0, width - 1],
                      [ 0, 1, 0],
                      [ 0, 0, 1]], dtype=np.float32)
        return F

    def get_corners(self, img, H):
        """
        이미지의 네 모서리를 H 변환을 적용해 계산합니다.
        """
        h, w = img.shape[:2]
        corners = np.array([[0, 0],
                            [w, 0],
                            [w, h],
                            [0, h]], dtype=np.float32).reshape(-1, 1, 2)
        warped = cv2.perspectiveTransform(corners, H)
        return warped.reshape(-1, 2)

    def stitch_panorama(self):
        if self.left_image is None or self.center_image is None or self.right_image is None:
            return None

        # 기준 이미지는 중앙 이미지 (그대로 유지)
        center_img = self.center_image
        left_img = self.left_image
        right_img = self.right_image

        h_center, w_center = center_img.shape[:2]

        # 왼쪽 이미지는 H_left_center를 그대로 사용 (flip하지 않음)
        H_left_final = self.H_left_center

        # 오른쪽 이미지는 flip한 후 호모그래피를 적용하고, 다시 flip하여 최종 결과를 만듭니다.
        w_right = right_img.shape[1]
        F_right = self.horizontal_flip_matrix(w_right)
        H_right_final = F_right @ self.H_center_right @ F_right

        # 각 이미지의 모서리 좌표를 구해서 전체 캔버스 크기를 결정합니다.
        corners_left = self.get_corners(left_img, H_left_final)
        corners_center = np.array([[0, 0],
                                   [w_center, 0],
                                   [w_center, h_center],
                                   [0, h_center]], dtype=np.float32)
        corners_right = self.get_corners(right_img, H_right_final)

        all_x = np.concatenate((corners_left[:, 0], corners_center[:, 0], corners_right[:, 0]))
        all_y = np.concatenate((corners_left[:, 1], corners_center[:, 1], corners_right[:, 1]))
        min_x, max_x = np.min(all_x), np.max(all_x)
        min_y, max_y = np.min(all_y), np.max(all_y)

        offset_x = -min_x if min_x < 0 else 0
        offset_y = -min_y if min_y < 0 else 0
        canvas_width = int(np.ceil(max_x - min_x))
        canvas_height = int(np.ceil(max_y - min_y))

        # Translation 행렬: 전체 이미지를 캔버스 내에 맞추기 위한 보정
        T = np.array([[1, 0, offset_x],
                      [0, 1, offset_y],
                      [0, 0, 1]], dtype=np.float32)

        # 각 이미지를 warpPerspective로 투영
        warped_left = cv2.warpPerspective(left_img, T @ H_left_final, (canvas_width, canvas_height))
        warped_center = cv2.warpPerspective(center_img, T, (canvas_width, canvas_height))
        warped_right = cv2.warpPerspective(right_img, T @ H_right_final, (canvas_width, canvas_height))

        # 파노라마 합성 (중앙 이미지를 우선, 그 외 빈 영역에 왼쪽/오른쪽 이미지 덮어쓰기)
        panorama = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)
        center_mask = (warped_center.sum(axis=2) > 0)
        panorama[center_mask] = warped_center[center_mask]

        left_mask = (warped_left.sum(axis=2) > 0) & (panorama.sum(axis=2) == 0)
        panorama[left_mask] = warped_left[left_mask]

        right_mask = (warped_right.sum(axis=2) > 0) & (panorama.sum(axis=2) == 0)
        panorama[right_mask] = warped_right[right_mask]

        return panorama

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            pano = self.stitch_panorama()
            if pano is not None:
                try:
                    pano_msg = self.bridge.cv2_to_imgmsg(pano, "bgr8")
                    self.panorama_pub.publish(pano_msg)
                except Exception as e:
                    rospy.logerr("Failed to publish panorama: " + str(e))
            rate.sleep()

if __name__ == "__main__":
    try:
        node = PanoramaStitcherROS()
        node.run()
    except rospy.ROSInterruptException:
        pass

