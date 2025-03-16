import cv2
import numpy as np
import time

class HomographyCalibrator:
    def __init__(self):
        self.left_cap = cv2.VideoCapture(0)    # 왼쪽 카메라
        self.center_cap = cv2.VideoCapture(4)  # 중앙 카메라 (기준)
        self.right_cap = cv2.VideoCapture(2)   # 오른쪽 카메라
        self.calibration_time = 30             # Calibration 시간 설정
        self.start_time = time.time()

    def calibrate_and_save_homographies(self):
        best_H_left = None
        best_H_right = None

        print("Starting calibration...")

        while time.time() - self.start_time < self.calibration_time:
            ret_left, left_image = self.left_cap.read()
            ret_center, center_image = self.center_cap.read()
            ret_right, right_image = self.right_cap.read()

            if not (ret_left and ret_center and ret_right):
                print("Failed to capture one or more images. Retrying...")
                time.sleep(0.1)
                continue

            # 왼쪽 카메라 → 중앙 카메라 호모그래피 계산
            H_left = self.compute_homography(left_image, center_image)
            if H_left is not None:
                best_H_left = H_left

            # 오른쪽 카메라 → 중앙 카메라 호모그래피 계산
            H_right = self.compute_homography(right_image, center_image)
            if H_right is not None:
                best_H_right = H_right

        # 호모그래피 행렬 저장
        if best_H_left is not None:
            np.save("homography_left.npy", best_H_left)
            print("Homography matrix for left saved to homography_left.npy")
        else:
            print("Failed to calculate homography for left.")

        if best_H_right is not None:
            np.save("homography_right.npy", best_H_right)
            print("Homography matrix for right saved to homography_right.npy")
        else:
            print("Failed to calculate homography for right.")

        self.release_resources()

    def compute_homography(self, image1, image2):
        try:
            gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

            akaze = cv2.AKAZE_create()
            keypoints1, descriptors1 = akaze.detectAndCompute(gray1, None)
            keypoints2, descriptors2 = akaze.detectAndCompute(gray2, None)

            if descriptors1 is None or descriptors2 is None:
                print("Failed to detect features in one or both images.")
                return None

            # 매칭 및 필터링
            matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
            knn_matches = matcher.knnMatch(descriptors1, descriptors2, k=2)

            good_matches = [m for m, n in knn_matches if m.distance < 0.75 * n.distance]

            if len(good_matches) < 10:
                print("Not enough good matches for homography computation. Matches found:", len(good_matches))
                return None

            # 매칭 점들 추출
            points1 = np.float32([keypoints1[m.queryIdx].pt for m in good_matches])
            points2 = np.float32([keypoints2[m.trainIdx].pt for m in good_matches])

            # 호모그래피 행렬 계산 (중앙 카메라 기준으로 변경됨)
            # points1은 중앙 카메라의 점들, points2는 다른 카메라의 점들
            H, mask = cv2.findHomography(points2, points1, cv2.RANSAC, 5.0)
            inlier_count = np.sum(mask)

            print(f"Homography computed with {inlier_count} inliers out of {len(good_matches)} matches.")
            return H

        except cv2.error as e:
            print(f"OpenCV error during homography computation: {e}")
            return None
        except Exception as e:
            print(f"Unexpected error during homography computation: {e}")
            return None

    def release_resources(self):
        self.left_cap.release()
        self.center_cap.release()
        self.right_cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrator = HomographyCalibrator()
    calibrator.calibrate_and_save_homographies()

