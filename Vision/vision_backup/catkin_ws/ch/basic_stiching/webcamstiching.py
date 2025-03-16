# ================================================================================
# Main Author: 이창현, 윤수빈
# Recently Modified Date: 2024-12-22 (V1.0)
# Dependency: multicam
# Description: 웹캠 2대를 스티칭 기법을 사용하여 파노라마 뷰로 제작. (30초간 호모그래피 최적화 후 고정) - ros 버전
# ================================================================================

# import cv2
# import numpy as np

# class PanoramaNode:
#     def __init__(self):
#         # Initialize video captures for left and right webcams
#         self.left_cap = cv2.VideoCapture(2)  # Adjust index as per your camera setup
#         self.right_cap = cv2.VideoCapture(0)  # Adjust index as per your camera setup

#     def process_images(self):
#         ret_left, left_image = self.left_cap.read()
#         ret_right, right_image = self.right_cap.read()

#         if not ret_left or not ret_right:
#             print("Failed to capture one or both images.")
#             return

#         # Show input images for debugging
#         cv2.imshow("Left Image", left_image)
#         cv2.imshow("Right Image", right_image)
#         cv2.waitKey(1)

#         panorama = self.make_panorama(left_image, right_image)
#         if panorama is not None:
#             cv2.imshow("Panorama", panorama)
#             cv2.waitKey(1)

#     def make_panorama(self, left_image, right_image):
#         try:
#             # Convert images to grayscale
#             gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
#             gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

#             # Detect AKAZE features and compute descriptors
#             akaze = cv2.AKAZE_create()
#             keypoints_left, descriptors_left = akaze.detectAndCompute(gray_left, None)
#             keypoints_right, descriptors_right = akaze.detectAndCompute(gray_right, None)

#             # Match features using KNN Matcher
#             matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
#             knn_matches = matcher.knnMatch(descriptors_left, descriptors_right, k=2)

#             # Apply Lowe's ratio test
#             good_matches = []
#             for m, n in knn_matches:
#                 if m.distance < 0.75 * n.distance:  # Ratio test threshold
#                     good_matches.append(m)

#             # Visualize matches
#             match_img = cv2.drawMatches(left_image, keypoints_left, right_image, keypoints_right, good_matches, None)
#             cv2.imshow("Matches", match_img)
#             cv2.waitKey(1)

#             # Debugging: Check number of matches
#             print(f"Number of Good Matches: {len(good_matches)}")
#             if len(good_matches) < 10:
#                 print("Not enough good matches to compute panorama.")
#                 return None

#             # Extract matched points
#             points_left = np.float32([keypoints_left[m.queryIdx].pt for m in good_matches])
#             points_right = np.float32([keypoints_right[m.trainIdx].pt for m in good_matches])

#             # Check for sufficient points
#             if len(points_left) < 4 or len(points_right) < 4:
#                 print("Not enough points for homography computation.")
#                 return None

#             # Compute homography with RANSAC
#             H, mask = cv2.findHomography(points_right, points_left, cv2.RANSAC, 5.0)
#             print(f"Homography Matrix: \n{H}")
#             if H is None:
#                 print("Homography computation failed.")
#                 return None

#             # Warp perspective
#             height = max(left_image.shape[0], right_image.shape[0])
#             width = left_image.shape[1] + right_image.shape[1]
#             result = cv2.warpPerspective(right_image, H, (width, height))
#             result[0:left_image.shape[0], 0:left_image.shape[1]] = left_image
#             return result

#         except Exception as e:
#             print(f"Error in make_panorama: {e}")
#             return None

# if __name__ == "__main__":
#     panorama_node = PanoramaNode()
#     while True:
#         panorama_node.process_images()
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     # Release video captures and close windows
#     panorama_node.left_cap.release()
#     panorama_node.right_cap.release()
#     cv2.destroyAllWindows()



import cv2
import numpy as np
import time

class PanoramaNode:
    def __init__(self):
        # Initialize video captures for left and right webcams
        self.left_cap = cv2.VideoCapture(0)  # Adjust index as per your camera setup
        self.right_cap = cv2.VideoCapture(2)  # Adjust index as per your camera setup
        self.best_H = None  # Store the best homography matrix
        self.calibration_time = 30  # Duration to calibrate in seconds
        self.start_time = time.time()

    def process_images(self):
        ret_left, left_image = self.left_cap.read()
        ret_right, right_image = self.right_cap.read()

        if not ret_left or not ret_right:
            print("Failed to capture one or both images.")
            return

        # Show input images for debugging
        cv2.imshow("Left Image", left_image)
        cv2.imshow("Right Image", right_image)
        cv2.waitKey(1)

        # During calibration, find the best homography matrix
        if time.time() - self.start_time < self.calibration_time:
            self.best_H = self.calibrate_homography(left_image, right_image)
        else:
            # Use the fixed homography matrix for stitching
            if self.best_H is not None:
                panorama = self.make_panorama_with_fixed_H(left_image, right_image)
                if panorama is not None:
                    cv2.imshow("Panorama", panorama)
                    cv2.waitKey(1)

    def calibrate_homography(self, left_image, right_image):
        try:
            gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

            # Detect AKAZE features and compute descriptors
            akaze = cv2.AKAZE_create()
            keypoints_left, descriptors_left = akaze.detectAndCompute(gray_left, None)
            keypoints_right, descriptors_right = akaze.detectAndCompute(gray_right, None)

            # Match features using KNN Matcher
            matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
            knn_matches = matcher.knnMatch(descriptors_left, descriptors_right, k=2)

            # Apply Lowe's ratio test
            good_matches = []
            for m, n in knn_matches:
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)

            # Debugging: Check number of matches
            print(f"Number of Good Matches (Calibration): {len(good_matches)}")
            if len(good_matches) < 10:
                print("Not enough good matches for homography calibration.")
                return self.best_H

            points_left = np.float32([keypoints_left[m.queryIdx].pt for m in good_matches])
            points_right = np.float32([keypoints_right[m.trainIdx].pt for m in good_matches])

            if len(points_left) < 4 or len(points_right) < 4:
                print("Not enough points for homography calibration.")
                return self.best_H

            H, _ = cv2.findHomography(points_right, points_left, cv2.RANSAC, 5.0)
            print(f"Calibrated Homography Matrix: \n{H}")
            return H

        except Exception as e:
            print(f"Error during homography calibration: {e}")
            return self.best_H

    def make_panorama_with_fixed_H(self, left_image, right_image):
        try:
            if self.best_H is None:
                print("Homography matrix is not calibrated.")
                return None

            height = max(left_image.shape[0], right_image.shape[0])
            width = left_image.shape[1] + right_image.shape[1]
            result = cv2.warpPerspective(right_image, self.best_H, (width, height))
            result[0:left_image.shape[0], 0:left_image.shape[1]] = left_image
            return result

        except Exception as e:
            print(f"Error during panorama stitching with fixed H: {e}")
            return None

if __name__ == "__main__":
    panorama_node = PanoramaNode()
    while True:
        panorama_node.process_images()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release video captures and close windows
    panorama_node.left_cap.release()
    panorama_node.right_cap.release()
    cv2.destroyAllWindows()
