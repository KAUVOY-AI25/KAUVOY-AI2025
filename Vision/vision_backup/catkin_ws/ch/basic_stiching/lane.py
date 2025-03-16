# # import rospy
# # import cv2
# # import numpy as np
# # from cv_bridge import CvBridge
# # from sensor_msgs.msg import Image

# # def convert_to_hls(image):
# #     return cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

# # def filter_colors(hls_image):
# #     white_lower = np.array([0, 170, 0], dtype=np.uint8)
# #     white_upper = np.array([255, 255, 255], dtype=np.uint8)
# #     yellow_lower = np.array([15, 30, 115], dtype=np.uint8)
# #     yellow_upper = np.array([35, 204, 255], dtype=np.uint8)

# #     white_mask = cv2.inRange(hls_image, white_lower, white_upper)
# #     yellow_mask = cv2.inRange(hls_image, yellow_lower, yellow_upper)

# #     combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
# #     return cv2.bitwise_and(hls_image, hls_image, mask=combined_mask)

# # def detect_edges(filtered_image):
# #     gray_image = cv2.cvtColor(filtered_image, cv2.COLOR_HLS2BGR)
# #     gray_image = cv2.cvtColor(gray_image, cv2.COLOR_BGR2GRAY)
# #     blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
# #     return cv2.Canny(blurred_image, 50, 150)

# # def region_of_interest(image):
# #     height, width = image.shape
# #     mask = np.zeros_like(image)

# #     # Define region of interest
# #     polygon = np.array([[
# #         (0, height),
# #         (100, height // 2),
# #         (width - 100, height // 2),
# #         (width, height)
# #     ]], np.int32)

# #     cv2.fillPoly(mask, polygon, 255)
# #     return cv2.bitwise_and(image, mask)

# # def hough_lines(edge_image):
# #     lines = cv2.HoughLinesP(edge_image, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=50)
# #     filtered_lines = []
# #     if lines is not None:
# #         for line in lines:
# #             x1, y1, x2, y2 = line[0]
# #             if abs(y2 - y1) > 30 and abs(x2 - x1) > 30:  # Filter out short or nearly horizontal/vertical lines
# #                 filtered_lines.append([[x1, y1, x2, y2]])
# #     return filtered_lines

# # def draw_lines(image, lines):
# #     line_image = np.zeros_like(image)
# #     if lines is not None:
# #         for line in lines:
# #             x1, y1, x2, y2 = line[0]
# #             cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 10)
# #     return line_image

# # def combine_images(original, lines, lane_area):
# #     lane_image = cv2.addWeighted(original, 0.8, lane_area, 0.3, 0)
# #     line_image = cv2.addWeighted(original, 0.8, lines, 1, 0)
# #     return lane_image, line_image

# # class LaneDetectionNode:
# #     def __init__(self):
# #         rospy.init_node('lane_detection_node', anonymous=True)

# #         self.bridge = CvBridge()
# #         rospy.Subscriber('/img_topic', Image, self.image_callback)

# #         self.lane_pub = rospy.Publisher('/lane_image', Image, queue_size=10)
# #         self.line_pub = rospy.Publisher('/line_image', Image, queue_size=10)

# #     def image_callback(self, msg):
# #         # Convert ROS Image to OpenCV format
# #         original_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

# #         # Process Image
# #         hls_image = convert_to_hls(original_image)
# #         filtered_image = filter_colors(hls_image)
# #         edges = detect_edges(filtered_image)
# #         cropped_edges = region_of_interest(edges)

# #         lines = hough_lines(cropped_edges)
# #         line_image = draw_lines(original_image, lines)

# #         # Fill lane region
# #         lane_area = cv2.fillPoly(np.zeros_like(line_image), np.array([[[0, 720], [200, 450], [1080, 450], [1280, 720]]]), (0, 0, 255))
# #         lane_image, line_only_image = combine_images(original_image, line_image, lane_area)

# #         # Publish results
# #         lane_msg = self.bridge.cv2_to_imgmsg(lane_image, 'bgr8')
# #         line_msg = self.bridge.cv2_to_imgmsg(line_only_image, 'bgr8')
        
# #         self.lane_pub.publish(lane_msg)
# #         self.line_pub.publish(line_msg)

# # if __name__ == '__main__':
# #     try:
# #         LaneDetectionNode()
# #         rospy.spin()
# #     except rospy.ROSInterruptException:
# #         pass



# import rospy
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image

# def convert_to_hls(image):
#     return cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

# def filter_colors(hls_image):
#     white_lower = np.array([0, 170, 0], dtype=np.uint8)  # Extended range for white
#     white_upper = np.array([255, 255, 255], dtype=np.uint8)
#     yellow_lower = np.array([15, 30, 115], dtype=np.uint8)
#     yellow_upper = np.array([35, 204, 255], dtype=np.uint8)

#     white_mask = cv2.inRange(hls_image, white_lower, white_upper)
#     yellow_mask = cv2.inRange(hls_image, yellow_lower, yellow_upper)

#     combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
#     return cv2.bitwise_and(hls_image, hls_image, mask=combined_mask)

# def bird_eye_view(image):
#     height, width = image.shape[:2]
#     src_points = np.float32([
#         [0, 1080], [800, 680], [1120, 680], [1920, 1080]
#     ])
#     dst_points = np.float32([
#         [0, 1080], [0, 0], [1920, 0], [1920, 1080]
#     ])
#     matrix = cv2.getPerspectiveTransform(src_points, dst_points)
#     return cv2.warpPerspective(image, matrix, (width, height))

# def detect_edges(filtered_image):
#     gray_image = cv2.cvtColor(filtered_image, cv2.COLOR_HLS2BGR)
#     gray_image = cv2.cvtColor(gray_image, cv2.COLOR_BGR2GRAY)
#     blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
#     return cv2.Canny(blurred_image, 50, 150)

# def region_of_interest(image):
#     height, width = image.shape
#     mask = np.zeros_like(image)

#     # Define region of interest
#     polygon = np.array([[
#         (0, height),
#         (100, height // 2),
#         (width - 100, height // 2),
#         (width, height)
#     ]], np.int32)

#     cv2.fillPoly(mask, polygon, 255)
#     return cv2.bitwise_and(image, mask)

# def hough_lines(edge_image):
#     lines = cv2.HoughLinesP(edge_image, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=50)
#     if lines is None:
#         return []
#     return lines

# def draw_lines(image, lines):
#     line_image = np.zeros_like(image)
#     if lines is not None:
#         for line in lines:
#             x1, y1, x2, y2 = line[0]
#             cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 10)
#     return line_image

# def combine_images(original, lines, lane_area):
#     lane_image = cv2.addWeighted(original, 0.8, lane_area, 0.3, 0)
#     line_image = cv2.addWeighted(original, 0.8, lines, 1, 0)
#     return lane_image, line_image

# class LaneDetectionNode:
#     def __init__(self):
#         rospy.init_node('lane_detection_node', anonymous=True)

#         self.bridge = CvBridge()
#         rospy.Subscriber('/img_topic', Image, self.image_callback)

#         self.lane_pub = rospy.Publisher('/lane_image', Image, queue_size=10)
#         self.line_pub = rospy.Publisher('/line_image', Image, queue_size=10)

#     def image_callback(self, msg):
#         # Convert ROS Image to OpenCV format
#         original_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

#         # Apply Bird Eye View Transformation
#         bird_eye_image = bird_eye_view(original_image)

#         # Process Image
#         hls_image = convert_to_hls(bird_eye_image)
#         filtered_image = filter_colors(hls_image)
#         #edges = detect_edges(filtered_image)
#         #cropped_edges = region_of_interest(edges)
#         cropped_edges = detect_edges(filtered_image)

#         lines = hough_lines(cropped_edges)
#         line_image = draw_lines(bird_eye_image, lines)

#         # Fill lane region
#         lane_area = cv2.fillPoly(np.zeros_like(line_image), np.array([[[0, 720], [200, 450], [1080, 450], [1280, 720]]]), (0, 0, 255))
#         lane_image, line_only_image = combine_images(bird_eye_image, line_image, lane_area)

#         # Publish results
#         lane_msg = self.bridge.cv2_to_imgmsg(lane_image, 'bgr8')
#         line_msg = self.bridge.cv2_to_imgmsg(line_only_image, 'bgr8')
        
#         self.lane_pub.publish(lane_msg)
#         self.line_pub.publish(line_msg)

# if __name__ == '__main__':
#     try:
#         LaneDetectionNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def convert_to_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

def filter_colors(hsv_image):
    white_lower = np.array([0, 0, 150], dtype=np.uint8)  # Extended range for white
    white_upper = np.array([180, 30, 255], dtype=np.uint8)
    yellow_lower = np.array([18, 97, 97], dtype=np.uint8)
    yellow_upper = np.array([30, 255, 255], dtype=np.uint8)

    white_mask = cv2.inRange(hsv_image, white_lower, white_upper)
    yellow_mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)

    combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
    return cv2.bitwise_and(hsv_image, hsv_image, mask=combined_mask)

def bird_eye_view(image):
    height, width = image.shape[:2]
    src_points = np.float32([
        [0, 1080], [800, 680], [1120, 680], [1920, 1080]
    ])
    dst_points = np.float32([
        [0, 1080], [0, 0], [1920, 0], [1920, 1080]
    ])
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    return cv2.warpPerspective(image, matrix, (width, height))

def detect_edges(filtered_image):
    gray_image = cv2.cvtColor(filtered_image, cv2.COLOR_HLS2BGR)
    gray_image = cv2.cvtColor(gray_image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
    return blurred_image, cv2.Canny(blurred_image, 100, 200)

def region_of_interest(image):
    height, width = image.shape
    mask = np.zeros_like(image)

    # Define region of interest
    polygon = np.array([[
        (0, height),
        (100, height // 2),
        (width - 100, height // 2),
        (width, height)
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    return cv2.bitwise_and(image, mask)

def hough_lines(edge_image):
    lines = cv2.HoughLinesP(edge_image, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=50)
    if lines is None:
        return []
    return lines

def draw_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 10)
    return line_image

def combine_images(original, lines, lane_area):
    lane_image = cv2.addWeighted(original, 0.8, lane_area, 0.3, 0)
    line_image = cv2.addWeighted(original, 0.8, lines, 1, 0)
    return lane_image, line_image

class LaneDetectionNode:
    def __init__(self):
        rospy.init_node('lane_detection_node', anonymous=True)

        self.bridge = CvBridge()
        rospy.Subscriber('/img_topic', Image, self.image_callback)

        self.lane_pub = rospy.Publisher('/lane_image', Image, queue_size=10)
        self.line_pub = rospy.Publisher('/line_image', Image, queue_size=10)
        self.canny_pub = rospy.Publisher('/canny', Image, queue_size=10)
        self.color_mask_pub = rospy.Publisher('/color_mask', Image, queue_size=10)
        self.blurred_pub = rospy.Publisher('/blurred', Image, queue_size=10)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        original_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Apply Bird Eye View Transformation
        bird_eye_image = bird_eye_view(original_image)

        # Process Image
        hsv_image = convert_to_hsv(bird_eye_image)
        filtered_image = filter_colors(hsv_image)

        # Publish Color Mask results
        color_mask_msg = self.bridge.cv2_to_imgmsg(filtered_image, 'bgr8')
        self.color_mask_pub.publish(color_mask_msg)

        blurred_image, edges = detect_edges(filtered_image)

        # Publish Blurred Image results
        blurred_msg = self.bridge.cv2_to_imgmsg(blurred_image, 'mono8')
        self.blurred_pub.publish(blurred_msg)

        # Publish Canny edge results
        canny_msg = self.bridge.cv2_to_imgmsg(edges, 'mono8')
        self.canny_pub.publish(canny_msg)

        # Region of interest
        cropped_edges = region_of_interest(edges)

        lines = hough_lines(cropped_edges)
        line_image = draw_lines(bird_eye_image, lines)

        # Fill lane region
        lane_area = cv2.fillPoly(np.zeros_like(line_image), np.array([[[0, 720], [200, 450], [1080, 450], [1280, 720]]]), (0, 0, 255))
        lane_image, line_only_image = combine_images(bird_eye_image, line_image, lane_area)

        # Publish results
        lane_msg = self.bridge.cv2_to_imgmsg(lane_image, 'bgr8')
        line_msg = self.bridge.cv2_to_imgmsg(line_only_image, 'bgr8')
        
        self.lane_pub.publish(lane_msg)
        self.line_pub.publish(line_msg)

if __name__ == '__main__':
    try:
        LaneDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
