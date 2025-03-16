import cv2
import numpy as np

class PanoramaStitcher:
    def __init__(self):
        # 카메라 초기화 및 속도 최적화
        self.left_cap = cv2.VideoCapture(0)
        self.center_cap = cv2.VideoCapture(2)
        self.right_cap = cv2.VideoCapture(4)

        # 카메라 속도 최적화 (프레임 크기 및 버퍼 설정)
        for cap in [self.left_cap, self.center_cap, self.right_cap]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)  # 프레임 속도 제한

        # 호모그래피 행렬 로드
        self.H_left_center = self.load_homography("homography_left_center.npy")
        self.H_center_right = self.load_homography("homography_center_right.npy")

    def load_homography(self, filename):
        try:
            return np.load(filename).astype(np.float32)  # float32로 변환해 연산 최적화
        except FileNotFoundError:
            print(f"Homography matrix file {filename} not found.")
            return None

    def stitch_panorama(self):
        if self.H_left_center is None or self.H_center_right is None:
            return

        # 카메라에서 이미지 캡처 (속도 최적화)
        ret_left, left_image = self.left_cap.read()
        ret_center, center_image = self.center_cap.read()
        ret_right, right_image = self.right_cap.read()

        if not ret_left or not ret_center or not ret_right:
            print("Failed to capture one or more images.")
            return

        # 왼쪽-중앙 스티칭
        left_image_flipped = cv2.flip(left_image, 1)  # 좌우 반전 유지
        center_image_flipped = cv2.flip(center_image, 1)  # 좌우 반전 유지
        height_left_center = max(left_image_flipped.shape[0], center_image_flipped.shape[0])
        width_left_center = left_image_flipped.shape[1] + center_image_flipped.shape[1]

        # 호모그래피 적용 (연산 속도 개선)
        stitched_left_center = cv2.warpPerspective(left_image_flipped, self.H_left_center, (width_left_center, height_left_center), flags=cv2.INTER_LINEAR)
        stitched_left_center[:center_image_flipped.shape[0], :center_image_flipped.shape[1]] = center_image_flipped

        # 중앙-오른쪽 스티칭
        height_center_right = max(center_image.shape[0], right_image.shape[0])
        width_center_right = center_image.shape[1] + right_image.shape[1]
        stitched_center_right = cv2.warpPerspective(right_image, self.H_center_right, (width_center_right, height_center_right), flags=cv2.INTER_LINEAR)
        stitched_center_right[:center_image.shape[0], :center_image.shape[1]] = center_image

        # 최종 병합 (NumPy 연산 최적화)
        height_final = max(stitched_left_center.shape[0], stitched_center_right.shape[0])
        width_final = stitched_left_center.shape[1] + stitched_center_right.shape[1] - center_image.shape[1]

        stitched_final = np.zeros((height_final, width_final, 3), dtype=np.uint8)
        stitched_final[:stitched_left_center.shape[0], :stitched_left_center.shape[1]] = stitched_left_center
        stitched_final[:stitched_center_right.shape[0], stitched_left_center.shape[1]:] = stitched_center_right[:, center_image.shape[1]:]

        # 왼쪽-중앙 부분만 좌우 반전 (불필요한 flip 제거)
        stitched_final[:, :stitched_left_center.shape[1]] = cv2.flip(stitched_final[:, :stitched_left_center.shape[1]], 1)

        # 최종 파노라마 출력 (GUI 속도 개선)
        cv2.imshow("Optimized Panorama", stitched_final)
        cv2.waitKey(10)  # 너무 빠르게 반복되지 않도록 조정

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




