import cv2
import numpy as np
import time

class HomographyCalibrator:
    def __init__(self):
        self.left_cap = cv2.VideoCapture(0)    # 왼쪽 카메라
        self.center_cap = cv2.VideoCapture(2)  # 가운데 카메라
        self.right_cap = cv2.VideoCapture(4)   # 오른쪽 카메라
        self.calibration_time = 30  # Calibration 시간 설정
        self.start_time = time.time()

    def calibrate_and_save_homographies(self):
        best_H_left_center = None
        best_H_center_right = None

        while time.time() - self.start_time < self.calibration_time:
            ret_left, left_image = self.left_cap.read()
            ret_center, center_image = self.center_cap.read()
            ret_right, right_image = self.right_cap.read()

            if not ret_left or not ret_center or not ret_right:
                print("Failed to capture one or more images.")
                continue

            # 왼쪽-가운데 호모그래피 계산
            H_left_center = self.compute_homography(left_image, center_image)
            if H_left_center is not None:
                best_H_left_center = H_left_center

            # 가운데-오른쪽 호모그래피 계산
            H_center_right = self.compute_homography(center_image, right_image)
            if H_center_right is not None:
                best_H_center_right = H_center_right

        # 호모그래피 행렬 저장
        if best_H_left_center is not None:
            np.save("homography_left_center.npy", best_H_left_center)
            print("Homography matrix between left and center saved to homography_left_center.npy")
        else:
            print("Failed to calculate homography between left and center.")

        if best_H_center_right is not None:
            np.save("homography_center_right.npy", best_H_center_right)
            print("Homography matrix between center and right saved to homography_center_right.npy")
        else:
            print("Failed to calculate homography between center and right.")

        self.left_cap.release()
        self.center_cap.release()
        self.right_cap.release()

    def compute_homography(self, image1, image2):
        try:
            gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

            akaze = cv2.AKAZE_create()
            keypoints1, descriptors1 = akaze.detectAndCompute(gray1, None)
            keypoints2, descriptors2 = akaze.detectAndCompute(gray2, None)

            matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
            knn_matches = matcher.knnMatch(descriptors1, descriptors2, k=2)

            good_matches = [m for m, n in knn_matches if m.distance < 0.75 * n.distance]

            if len(good_matches) < 10:
                print("Not enough good matches for homography computation.")
                return None

            points1 = np.float32([keypoints1[m.queryIdx].pt for m in good_matches])
            points2 = np.float32([keypoints2[m.trainIdx].pt for m in good_matches])

            H, _ = cv2.findHomography(points2, points1, cv2.RANSAC, 5.0)
            return H

        except Exception as e:
            print(f"Error during homography computation: {e}")
            return None

if __name__ == "__main__":
    calibrator = HomographyCalibrator()
    calibrator.calibrate_and_save_homographies()

