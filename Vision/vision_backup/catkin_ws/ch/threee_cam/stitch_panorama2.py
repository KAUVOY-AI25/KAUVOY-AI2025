import cv2
import numpy as np

class PanoramaStitcher:
    def __init__(self):
        self.left_cap = cv2.VideoCapture(0)    # 왼쪽 카메라
        self.center_cap = cv2.VideoCapture(4)  # 가운데 카메라
        self.right_cap = cv2.VideoCapture(2)   # 오른쪽 카메라
        self.H_left_center = self.load_homography("homography_left_center.npy")
        self.H_center_right = self.load_homography("homography_center_right.npy")

    def load_homography(self, filename):
        try:
            H = np.load(filename)  # 저장해둔 호모그래피 행렬 파일 로드
            print(f"Loaded homography matrix from {filename}.")
            return H
        except FileNotFoundError:
            print(f"Homography matrix file {filename} not found. Please run calibration first.")
            return None

    def stitch_panorama(self):
        if self.H_left_center is None or self.H_center_right is None:
            return

        ret_left, left_image = self.left_cap.read()
        ret_center, center_image = self.center_cap.read()
        ret_right, right_image = self.right_cap.read()

        if not ret_left or not ret_center or not ret_right:
            print("Failed to capture one or more images.")
            return

        # 캔버스 크기 설정: 넉넉하게 중앙 이미지와 주변 이미지를 모두 포함할 수 있도록 설정
        canvas_height = max(left_image.shape[0], center_image.shape[0], right_image.shape[0])
        canvas_width = left_image.shape[1] + center_image.shape[1] + right_image.shape[1]
        canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

        # 중앙 이미지를 캔버스의 중앙에 배치
        center_x_offset = left_image.shape[1]
        canvas[0:center_image.shape[0], center_x_offset:center_x_offset + center_image.shape[1]] = center_image

        # 왼쪽 이미지를 중심 카메라 좌표계에 맞게 변환 및 합치기
        stitched_left = cv2.warpPerspective(left_image, self.H_left_center, (canvas_width, canvas_height))
        canvas = np.where(stitched_left > 0, stitched_left, canvas)

        # 오른쪽 이미지를 중심 카메라 좌표계에 맞게 변환 및 합치기
        stitched_right = cv2.warpPerspective(right_image, self.H_center_right, (canvas_width, canvas_height))
        canvas = np.where(stitched_right > 0, stitched_right, canvas)

        # 결과 출력
        cv2.imshow("Panorama", canvas)
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

