import cv2
import numpy as np
import time

class HomographyCalibrator:
    def __init__(self):
        self.left_cap = cv2.VideoCapture(0)    # 왼쪽 카메라
        self.center_cap = cv2.VideoCapture(2)  # 가운데 카메라
        self.calibration_time = 30  # Calibration 시간 설정
        self.start_time = time.time()

    def calibrate_and_save_homographies(self):
        best_H_left_center = None

        while time.time() - self.start_time < self.calibration_time:
            ret_left, left_image = self.left_cap.read()
            ret_center, center_image = self.center_cap.read()

            if not ret_left or not ret_center:
                print("Failed to capture one or more images.")
                continue

            # 왼쪽-가운데 호모그래피 계산
            H_left_center = self.compute_homography(left_image, center_image)
            if H_left_center is not None:
                best_H_left_center = H_left_center

        # 호모그래피 행렬 저장 및 역행렬 적용
        if best_H_left_center is not None:
            # 역행렬 계산
            H_inverse = np.linalg.inv(best_H_left_center)

            # 역행렬 저장
            np.save("homography_left_center.npy", H_inverse)
            print("Inverse homography matrix (center to left) saved to homography_center_to_left.npy")
        else:
            print("Failed to calculate homography between left and center.")

        self.left_cap.release()
        self.center_cap.release()

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

            # 호모그래피 계산: 왼쪽 -> 가운데 변환 행렬
            H, _ = cv2.findHomography(points1, points2, cv2.RANSAC, 5.0)
            return H

        except Exception as e:
            print(f"Error during homography computation: {e}")
            return None

if __name__ == "__main__":
    calibrator = HomographyCalibrator()
    calibrator.calibrate_and_save_homographies()

