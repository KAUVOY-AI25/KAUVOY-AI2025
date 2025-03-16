import cv2
import numpy as np
import time

class InvertedHomographyCalibrator:
    def __init__(self):
        self.left_cap = cv2.VideoCapture(0)    # 왼쪽 카메라
        self.center_cap = cv2.VideoCapture(2)  # 가운데 카메라
        self.calibration_time = 30  # Calibration 시간 설정
        self.start_time = time.time()

    def calibrate_and_save_homography(self):
        best_H_left_center = None

        while time.time() - self.start_time < self.calibration_time:
            ret_left, left_image = self.left_cap.read()
            ret_center, center_image = self.center_cap.read()

            if not ret_left or not ret_center:
                print("Failed to capture one or more images.")
                continue

            # **1. 카메라 이미지를 좌우반전**
            left_image_flipped = cv2.flip(left_image, 1)  # 왼쪽 이미지 좌우반전
            center_image_flipped = cv2.flip(center_image, 1)  # 중앙 이미지 좌우반전

            # **2. 반전된 이미지로 오른쪽 호모그래피 방식 적용**
            # 중앙-오른쪽과 동일하게 호모그래피 계산
            H_left_center = self.compute_homography(center_image_flipped, left_image_flipped)
            if H_left_center is not None:
                best_H_left_center = H_left_center

        # 호모그래피 행렬 저장
        if best_H_left_center is not None:
            np.save("homography_inverted_left_center.npy", best_H_left_center)
            print("Homography matrix between inverted left and center saved to homography_inverted_left_center.npy")
        else:
            print("Failed to calculate homography between inverted left and center.")

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

            # 중앙-오른쪽과 동일한 방향으로 호모그래피 계산
            H, _ = cv2.findHomography(points1, points2, cv2.RANSAC, 5.0)
            return H

        except Exception as e:
            print(f"Error during homography computation: {e}")
            return None

if __name__ == "__main__":
    calibrator = InvertedHomographyCalibrator()
    calibrator.calibrate_and_save_homography()

