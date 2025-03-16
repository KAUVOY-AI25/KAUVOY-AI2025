#!/usr/bin/env python3

import rospy
from vision_msgs.msg import Detection2DArray

def detection_callback(msg):
    for detection in msg.detections:
        for result in detection.results:
            # Class ID and confidence score
            class_id = result.id
            confidence_score = result.score

            # Bounding box coordinates
            bbox_center_x = detection.bbox.center.x
            bbox_center_y = detection.bbox.center.y
            bbox_width = detection.bbox.size_x
            bbox_height = detection.bbox.size_y

            # Assuming you have a predefined mapping of class IDs to names
            class_names = {
                0: "Red Cone",
                1: "Blue Cone",
                # Add more class IDs and names as needed
            }

            class_name = class_names.get(class_id, "Unknown Class")

            print(f"Detected Object: {class_name}")
            print(f"Confidence Score: {confidence_score}")
            print(f"Bounding Box Center: ({bbox_center_x}, {bbox_center_y})")
            print(f"Bounding Box Size: {bbox_width} x {bbox_height}")
            print("---------------------------------")

def listener():
    rospy.init_node('object_detection_listener', anonymous=True)
    rospy.Subscriber('/yolov9_detect', Detection2DArray, detection_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()