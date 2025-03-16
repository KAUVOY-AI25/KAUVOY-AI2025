import cv2
import numpy as np

class PanoramaStitcher:
    def __init__(self):
        self.left_cap = cv2.VideoCapture(0)  # 카메라 번호 설정 
        self.right_cap = cv2.VideoCapture(2)  # 카메라 번호 설정 
        self.H = self.load_homography()

    def load_homography(self):
        try:
            H = np.load("homography_matrix.npy") # 저장해둔 호모그래피 행렬 주소 
            print("Loaded homography matrix from file.")
            return H
        except FileNotFoundError:
            print("Homography matrix file not found. Please run calibration first.")
            return None

    def stitch_panorama(self):
        if self.H is None:
            return

        ret_left, left_image = self.left_cap.read()
        ret_right, right_image = self.right_cap.read()

        if not ret_left or not ret_right:
            print("Failed to capture one or both images.")
            return

        height = max(left_image.shape[0], right_image.shape[0])
        width = left_image.shape[1] + right_image.shape[1]
        result = cv2.warpPerspective(right_image, self.H, (width, height))
        result[0:left_image.shape[0], 0:left_image.shape[1]] = left_image

        cv2.imshow("Panorama", result)
        cv2.waitKey(1)

    def release_resources(self):
        self.left_cap.release()
        self.right_cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    stitcher = PanoramaStitcher()
    while True:
        stitcher.stitch_panorama()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    stitcher.release_resources()
