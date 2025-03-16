import cv2
import numpy as np

class PanoramaStitcherLeftCenter:
    def __init__(self):
        self.left_cap = cv2.VideoCapture(0)    # 왼쪽 카메라
        self.center_cap = cv2.VideoCapture(2)  # 가운데 카메라
        self.H_left_center = self.load_homography("homography_left_center.npy")

    def load_homography(self, filename):
        try:
            H = np.load(filename)  # 저장된 호모그래피 행렬 파일 로드
            print(f"Loaded homography matrix from {filename}.")
            return H
        except FileNotFoundError:
            print(f"Homography matrix file {filename} not found. Please run calibration first.")
            return None

    def stitch_panorama(self):
        if self.H_left_center is None:
            return

        # 카메라에서 이미지 캡처
        ret_left, left_image = self.left_cap.read()
        ret_center, center_image = self.center_cap.read()

        if not ret_left or not ret_center:
            print("Failed to capture one or more images.")
            return

        # 좌우 반전 처리
        left_image_flipped = cv2.flip(left_image, 1)
        center_image_flipped = cv2.flip(center_image, 1)

        # 호모그래피를 이용하여 왼쪽 이미지를 변환 후 병합
        height_final = max(left_image.shape[0], center_image.shape[0])
        width_final = left_image.shape[1] + center_image.shape[1]
        stitched_final = cv2.warpPerspective(left_image_flipped, self.H_left_center, (width_final, height_final))
        stitched_final[0:center_image.shape[0], 0:center_image.shape[1]] = center_image_flipped

        # 최종 결과를 다시 좌우 반전
        stitched_final = cv2.flip(stitched_final, 1)

        # 최종 파노라마 출력
        cv2.imshow("Panorama Left-Center", stitched_final)
        cv2.waitKey(1)

    def release_resources(self):
        self.left_cap.release()
        self.center_cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    stitcher = PanoramaStitcherLeftCenter()
    while True:
        stitcher.stitch_panorama()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    stitcher.release_resources()

