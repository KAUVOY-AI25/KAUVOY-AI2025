import cv2
import numpy as np

class PanoramaStitcher:
    def __init__(self):
        self.left_cap = cv2.VideoCapture(0)    # 왼쪽 카메라
        self.center_cap = cv2.VideoCapture(4)  # 가운데 카메라 (기준)
        self.right_cap = cv2.VideoCapture(2)   # 오른쪽 카메라
        self.H_left = self.load_homography("homography_left_center.npy")     # 왼쪽 -> 중앙
        self.H_right = self.load_homography("homography_center_right.npy")   # 오른쪽 -> 중앙

    def load_homography(self, filename):
        try:
            H = np.load(filename)
            print(f"Loaded homography matrix from {filename}.")
            return H
        except FileNotFoundError:
            print(f"Homography matrix file {filename} not found. Please run calibration first.")
            return None

    def stitch_panorama(self):
        if self.H_left is None or self.H_right is None:
            return

        # 카메라로부터 이미지 캡처
        ret_left, left_image = self.left_cap.read()
        ret_center, center_image = self.center_cap.read()
        ret_right, right_image = self.right_cap.read()

        if not (ret_left and ret_center and ret_right):
            print("Failed to capture one or more images.")
            return

        # 중앙 이미지를 중심으로 설정
        panorama_height = max(left_image.shape[0], center_image.shape[0], right_image.shape[0])
        panorama_width = left_image.shape[1] + center_image.shape[1] + right_image.shape[1]
        panorama = np.zeros((panorama_height, panorama_width, 3), dtype=np.uint8)

        # 중앙 이미지를 가운데 배치
        center_offset_x = left_image.shape[1]
        panorama[0:center_image.shape[0], center_offset_x:center_offset_x + center_image.shape[1]] = center_image

        # 왼쪽 이미지를 중앙 기준으로 변환 후 결합
        transformed_left = cv2.warpPerspective(left_image, self.H_left, (panorama_width, panorama_height))
        panorama = cv2.addWeighted(panorama, 1.0, transformed_left, 1.0, 0)

        # 오른쪽 이미지를 중앙 기준으로 변환 후 결합
        transformed_right = cv2.warpPerspective(right_image, self.H_right, (panorama_width, panorama_height))
        panorama = cv2.addWeighted(panorama, 1.0, transformed_right, 1.0, 0)

        # 결과 파노라마 출력
        cv2.imshow("Panorama", panorama)
        cv2.waitKey(1)

    def release_resources(self):
        self.left_cap.release()
        self.center_cap.release()
        self.right_cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    stitcher = PanoramaStitcher()
    while True:
        stitcher.stitch_panorama()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    stitcher.release_resources()

