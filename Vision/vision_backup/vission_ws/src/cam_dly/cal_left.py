import cv2
import numpy as np
import time

class PanoramaCalibrator:
    def __init__(self):
        self.left_cap = cv2.VideoCapture(0)  # 왼쪽 카메라
        self.center_cap = cv2.VideoCapture(2) # 가운데 카메라
        self.right_cap = cv2.VideoCapture(4)   # 오른쪽 카메라
        self.akaze = cv2.AKAZE_create()        # AKAZE 알고리즘 사용
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # BFMatcher 사용

    def capture_images_left_center(self):
        # 카메라에서 이미지 캡처 (왼쪽과 중앙)
        ret_left, left_image = self.left_cap.read()
        ret_center, center_image = self.center_cap.read()

        if not ret_left or not ret_center:
            print("Failed to capture one or more images (left or center).")
            return None, None

        # 좌우 반전 처리
        left_image_flipped = cv2.flip(left_image, 1)
        center_image_flipped = cv2.flip(center_image, 1)
        return left_image_flipped, center_image_flipped

    def capture_images_center_right(self):
        # 카메라에서 이미지 캡처 (중앙과 오른쪽)
        ret_center, center_image = self.center_cap.read()
        ret_right, right_image = self.right_cap.read()

        if not ret_center or not ret_right:
            print("Failed to capture one or more images (center or right).")
            return None, None

        return center_image, right_image

    def compute_and_save_homography(self, output_file, capture_images_function):
        start_time = time.time()
        best_matches = []
        best_H = None
        timeout = 30  # 30초 동안 반복

        while time.time() - start_time < timeout:
            image1, image2 = capture_images_function()
            if image1 is None or image2 is None:
                continue

            # 특징점 추출
            kp1, des1 = self.akaze.detectAndCompute(image1, None)
            kp2, des2 = self.akaze.detectAndCompute(image2, None)

            if des1 is None or des2 is None:
                print("Failed to detect features in one or more images.")
                continue

            # 특징점 매칭
            matches = self.matcher.match(des1, des2)
            matches = sorted(matches, key=lambda x: x.distance)

            if len(matches) > len(best_matches):
                best_matches = matches
                src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
                best_H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        if best_H is not None:
            # 행렬 저장
            np.save(output_file, best_H)
            print(f"Homography matrix saved to {output_file}.")
        else:
            print("Failed to compute a valid homography matrix within the time limit.")

    def release_resources(self):
        self.left_cap.release()
        self.center_cap.release()
        self.right_cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrator = PanoramaCalibrator()
    print("Calculating left-center homography...")
    calibrator.compute_and_save_homography("homography_left_center.npy", calibrator.capture_images_left_center)

    print("Calculating center-right homography...")
    calibrator.compute_and_save_homography("homography_center_right.npy", calibrator.capture_images_center_right)

    calibrator.release_resources()

