import cv2
import numpy as np
import time

class HomographyCalibrator:
    def __init__(self):
        self.left_cap = cv2.VideoCapture(0)  # 카메라 번호 설정
        self.right_cap = cv2.VideoCapture(4)  # 카메라 번호 설정
        self.calibration_time = 30  # Calibration 시간 설정
        self.start_time = time.time()

    def calibrate_and_save_homography(self):
        best_H = None
        while time.time() - self.start_time < self.calibration_time:
            ret_left, left_image = self.left_cap.read()
            ret_right, right_image = self.right_cap.read()

            if not ret_left or not ret_right:
                print("Failed to capture one or both images.")
                continue

            # Calculate homography
            H = self.compute_homography(left_image, right_image)
            if H is not None:
                best_H = H

        if best_H is not None:
            np.save("homography_matrix.npy", best_H)  
            print("Homography matrix saved to homography_matrix.npy")   # 호모그래피 행렬 저장 주소 설정
        else:
            print("Failed to calculate homography.")

        self.left_cap.release()
        self.right_cap.release()

    def compute_homography(self, left_image, right_image):
        try:
            gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

            akaze = cv2.AKAZE_create()
            keypoints_left, descriptors_left = akaze.detectAndCompute(gray_left, None)
            keypoints_right, descriptors_right = akaze.detectAndCompute(gray_right, None)

            matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
            knn_matches = matcher.knnMatch(descriptors_left, descriptors_right, k=2)

            good_matches = [m for m, n in knn_matches if m.distance < 0.75 * n.distance]

            if len(good_matches) < 10:
                print("Not enough good matches for homography computation.")
                return None

            points_left = np.float32([keypoints_left[m.queryIdx].pt for m in good_matches])
            points_right = np.float32([keypoints_right[m.trainIdx].pt for m in good_matches])

            H, _ = cv2.findHomography(points_right, points_left, cv2.RANSAC, 5.0)
            return H

        except Exception as e:
            print(f"Error during homography computation: {e}")
            return None


if __name__ == "__main__":
    calibrator = HomographyCalibrator()
    calibrator.calibrate_and_save_homography()
