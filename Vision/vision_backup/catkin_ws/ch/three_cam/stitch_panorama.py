import cv2
import numpy as np

class PanoramaStitcher:
    def __init__(self):
        self.left_cap = cv2.VideoCapture(0)    # 왼쪽 카메라
        self.center_cap = cv2.VideoCapture(4)  # 중앙 카메라
        self.right_cap = cv2.VideoCapture(2)   # 오른쪽 카메라

        # 미리 저장된 호모그래피 행렬 로드
        self.H_left = self.load_homography("homography_left.npy")
        self.H_right = self.load_homography("homography_right.npy")

    def load_homography(self, file_path):
        try:
            H = np.load(file_path)
            print(f"Loaded homography matrix from {file_path}.")
            return H
        except FileNotFoundError:
            print(f"Homography matrix file {file_path} not found. Please run calibration first.")
            return None

    def stitch_panorama(self):
        if self.H_left is None or self.H_right is None:
            print("Homography matrices are not loaded properly.")
            return

        # 각 카메라로부터 이미지 캡처
        ret_left, left_image = self.left_cap.read()
        ret_center, center_image = self.center_cap.read()
        ret_right, right_image = self.right_cap.read()

        if not (ret_left and ret_center and ret_right):
            print("Failed to capture one or more images.")
            return

        # 중앙 이미지를 기준으로 오른쪽 이미지 변환
        height = max(center_image.shape[0], right_image.shape[0])
        width = center_image.shape[1] + right_image.shape[1]
        panorama_right = cv2.warpPerspective(right_image, self.H_right, (width, height))
        panorama_right[0:center_image.shape[0], 0:center_image.shape[1]] = center_image

        # 중앙+오른쪽 파노라마에 왼쪽 이미지 변환
        height = max(panorama_right.shape[0], left_image.shape[0])
        width = panorama_right.shape[1] + left_image.shape[1]
        final_panorama = cv2.warpPerspective(left_image, self.H_left, (width, height))
        final_panorama[0:panorama_right.shape[0], left_image.shape[1]:] = panorama_right

        # 결과 파노라마 출력
        cv2.imshow("Panorama", final_panorama)
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

