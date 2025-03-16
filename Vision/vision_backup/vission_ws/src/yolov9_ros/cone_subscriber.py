# #!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import PointCloud

# def cone_callback(msg):
#     Cone_Sign_states = {
#         "cone_blue": 0,
#         "cone_yellow": 1
#     }
    
#     global Cone_Sign
    
#     for channel in msg.channels:
#         if channel.name in Cone_Sign_states:
#             Cone_Sign = Cone_Sign_states[channel.name]
#             print(f"Cone Color: {Cone_Sign} {channel.name}")
#             #print("Confidence Score:", channel.values[0])
#             print("Bounding Box Coordinates (x1, y1, x2, y2):", channel.values[1:5])


# def listener():
#     rospy.init_node('object_detection_listener', anonymous=True)
#     rospy.Subscriber('/yolov9_detect', PointCloud, cone_callback)
#     rospy.spin()


# if __name__ == '__main__':
#     listener()

    
#2차
#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import PointCloud

# # Define global variables
# Cone_Sign = None
# yolo_bboxes = []

# def yolo_callback(msg):
#     global yolo_bboxes
#     yolo_bboxes.clear()

#     Cone_Sign_states = {
#         "cone_blue": 0,
#         "cone_yellow": 1
#     }
    
#     global Cone_Sign

#     for channel in msg.channels:
#         if len(channel.values) >= 5:  # Ensure there is sufficient data
#             bbox = {
#                 'x1': channel.values[1],
#                 'y1': channel.values[2],
#                 'x2': channel.values[3],
#                 'y2': channel.values[4],
#                 'confidence': channel.values[0],
#                 'label': channel.name
#             }
#             yolo_bboxes.append(bbox)
#             rospy.loginfo(f"Processed bounding box for {channel.name}")

#             if channel.name in Cone_Sign_states:
#                 Cone_Sign = Cone_Sign_states[channel.name]
#                 #print(f"Cone Color: {Cone_Sign} ({channel.name})")
#                 #print(f"Bounding Box Coordinates (x1, y1, x2, y2): {channel.values[1:5]}")
#         #else:
#             #rospy.logwarn(f"Skipping channel {channel.name} due to insufficient data: {channel.values}")


# def obstacle_callback(msg):
#     global yolo_bboxes
#     x, y, z, pixel_x, pixel_y, distance = msg.data
#     for bbox in yolo_bboxes:
#         if bbox['x1'] <= pixel_x <= bbox['x2'] and bbox['y1'] <= pixel_y <= bbox['y2']:
#             print(f"Obstacle within bounding box: Label = {bbox['label']}, Confidence = {bbox['confidence']}")
#             print(f"Obstacle Coordinates: x = {x}, y = {y}")
#             print(f"Distance: {distance} meters")
#             #print(f"Pixel Coordinates: x = {pixel_x}, y = {pixel_y}")

# def listener():
#     rospy.init_node('object_detection_listener', anonymous=True)
#     rospy.Subscriber('/yolov9_detect', PointCloud, yolo_callback)
#     rospy.Subscriber('/obstacle_info', Float32MultiArray, obstacle_callback)
#     rospy.spin()

# if __name__ == '__main__':
#     listener()

#3차

#!/usr/bin/env python3



# import rospy
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import PointCloud
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# import threading

# # Define global variables
# Cone_Sign = None
# yolo_bboxes = []
# obstacles = []
# obstacles_lock = threading.Lock()

# # Initialize plot and plot elements
# fig, ax = plt.subplots()

# def init():
#     ax.set_xlim(-10, 10)
#     ax.set_ylim(-10, 10)
#     ax.grid(True)
#     ax.set_title('Obstacle Positions')
#     ax.set_xlabel('Y')
#     ax.set_ylabel('X')
#     ax.invert_xaxis()
#     ax.plot(0, 0, 'ro', label='LiDAR Position')
#     ax.legend()
#     return []

# def update(frame):
#     ax.clear()
#     init()

#     # Lock the obstacles list during update
#     with obstacles_lock:
#         for x, y, color in obstacles:
#             if color == 1:
#                 ax.plot(y, x, 'yo')  # Plot yellow for color 1 (yellow cone)
#             elif color == 2:
#                 ax.plot(y, x, 'bo')  # Plot blue for color 2 (blue cone)

#     return []

# def yolo_callback(msg):
#     global yolo_bboxes
#     yolo_bboxes.clear()

#     Cone_Sign_states = {
#         "cone_blue": 2,  # 2 represents blue
#         "cone_red": 1  # 1 represents yellow
#     }
    
#     global Cone_Sign

#     for channel in msg.channels:
#         if len(channel.values) >= 5:  # Ensure there is sufficient data
#             bbox = {
#                 'x1': channel.values[1],
#                 'y1': channel.values[2],
#                 'x2': channel.values[3],
#                 'y2': channel.values[4],
#                 'confidence': channel.values[0],
#                 'label': channel.name
#             }
#             yolo_bboxes.append(bbox)
#             rospy.loginfo(f"Processed bounding box for {channel.name}")

#             if channel.name in Cone_Sign_states:
#                 Cone_Sign = Cone_Sign_states[channel.name]

# def obstacle_callback(msg):
#     global yolo_bboxes
#     x, y, z, pixel_x, pixel_y, distance = msg.data
#     for bbox in yolo_bboxes:
#         if bbox['x1'] <= pixel_x <= bbox['x2'] and bbox['y1'] <= pixel_y <= bbox['y2']:
#             print(f"Obstacle within bounding box: Label = {bbox['label']}, Confidence = {bbox['confidence']}")
#             print(f"Obstacle Coordinates: x = {x}, y = {y}")
#             print(f"Distance: {distance} meters")

#             # Determine the color based on the type of cone
#             color = 0  # Default color if not a known cone type
#             if bbox['label'] == "cone_blue":
#                 color = 2  # Blue
#             elif bbox['label'] == "cone_red":
#                 color = 1  # Yellow

#             # Add obstacle's x, y coordinates and color to the list
#             with obstacles_lock:
#                 obstacles.append((x, y, color))

# def listener():
#     rospy.init_node('object_detection_listener', anonymous=True)
#     rospy.Subscriber('/yolov9_detect', PointCloud, yolo_callback)
#     rospy.Subscriber('/obstacle_info', Float32MultiArray, obstacle_callback)

#     ani = FuncAnimation(fig, update, init_func=init, blit=False, interval=50, cache_frame_data=False)
#     plt.show()

#     rospy.spin()

# if __name__ == '__main__':
#     listener()

#5차
#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import PointCloud
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# import threading
# import time

# # Define global variables
# Cone_Sign = None
# yolo_bboxes = []
# obstacles = []
# obstacles_lock = threading.Lock()

# # Initialize plot and plot elements
# fig, ax = plt.subplots()

# # Timeout duration in seconds for considering an obstacle as outdated
# timeout_duration = 0.4

# def init():
#     ax.set_xlim(-10, 10)
#     ax.set_ylim(-10, 10)
#     ax.grid(True)
#     ax.set_title('Obstacle Positions')
#     ax.set_xlabel('Y')
#     ax.set_ylabel('X')
#     ax.invert_xaxis()
#     ax.plot(0, 0, 'ro', label='LiDAR Position')
#     ax.legend()
#     return []

# def update(frame):
#     current_time = time.time()
#     ax.clear()
#     init()

#     # Lock the obstacles list during update
#     with obstacles_lock:
#         # Filter out old obstacles
#         global obstacles
#         obstacles = [(x, y, color, timestamp) for (x, y, color, timestamp) in obstacles 
#                      if current_time - timestamp < timeout_duration]

#         # Plot current obstacles
#         for x, y, color, _ in obstacles:
#             if color == 1:
#                 ax.plot(y, x, 'yo', label='Yellow Cone')  # Plot yellow for color 1 (yellow cone)
#             elif color == 2:
#                 ax.plot(y, x, 'bo', label='Blue Cone')  # Plot blue for color 2 (blue cone)

#     ax.legend()

#     return []

# def yolo_callback(msg):
#     global yolo_bboxes
#     yolo_bboxes.clear()

#     Cone_Sign_states = {
#         "cone_blue": 2,  # 2 represents blue
#         "cone_red": 1  # 1 represents yellow
#     }
    
#     global Cone_Sign

#     for channel in msg.channels:
#         if len(channel.values) >= 5:  # Ensure there is sufficient data
#             bbox = {
#                 'x1': channel.values[1],
#                 'y1': channel.values[2],
#                 'x2': channel.values[3],
#                 'y2': channel.values[4],
#                 'confidence': channel.values[0],
#                 'label': channel.name
#             }
#             yolo_bboxes.append(bbox)
#             rospy.loginfo(f"Processed bounding box for {channel.name}")

#             if channel.name in Cone_Sign_states:
#                 Cone_Sign = Cone_Sign_states[channel.name]

# def obstacle_callback(msg):
#     global yolo_bboxes
#     x, y, z, pixel_x, pixel_y, distance = msg.data
#     current_time = time.time()

#     for bbox in yolo_bboxes:
#         if bbox['x1'] <= pixel_x <= bbox['x2'] and bbox['y1'] <= pixel_y <= bbox['y2']:
#             print(f"Obstacle within bounding box: Label = {bbox['label']}, Confidence = {bbox['confidence']}")
#             print(f"Obstacle Coordinates: x = {x}, y = {y}")
#             print(f"Distance: {distance} meters")

#             # Determine the color based on the type of cone
#             color = 0  # Default color if not a known cone type
#             if bbox['label'] == "cone_blue":
#                 color = 2  # Blue
#             elif bbox['label'] == "cone_red":
#                 color = 1  # Yellow

#             # Add obstacle's x, y coordinates, color, and current timestamp to the list
#             with obstacles_lock:
#                 obstacles.append((x, y, color, current_time))

# def listener():
#     rospy.init_node('object_detection_listener', anonymous=True)
#     rospy.Subscriber('/yolov9_detect', PointCloud, yolo_callback)
#     rospy.Subscriber('/obstacle_info', Float32MultiArray, obstacle_callback)

#     ani = FuncAnimation(fig, update, init_func=init, blit=False, interval=50, cache_frame_data=False)
#     plt.show()

#     rospy.spin()

# if __name__ == '__main__':
#     listener()



#6차
#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import PointCloud
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# import threading
# import time

# # Define global variables
# Cone_Sign = None
# yolo_bboxes = []
# obstacles = {}
# obstacles_lock = threading.Lock()

# # Initialize plot and plot elements
# fig, ax = plt.subplots()

# # Timeout duration in seconds for considering an obstacle as outdated
# timeout_duration = 0.4

# def init():
#     ax.set_xlim(-10, 10)
#     ax.set_ylim(-10, 10)
#     ax.grid(True)
#     ax.set_title('Obstacle Positions')
#     ax.set_xlabel('Y')
#     ax.set_ylabel('X')
#     ax.invert_xaxis()
#     ax.plot(0, 0, 'ro', label='LiDAR Position')
#     ax.legend()
#     return []

# def update(frame):
#     current_time = time.time()
#     ax.clear()
#     init()

#     # Lock the obstacles dictionary during update
#     with obstacles_lock:
#         # Remove outdated obstacles
#         obstacles_to_remove = []
#         for obs_id, data in obstacles.items():
#             if current_time - data['timestamp'] > timeout_duration:
#                 obstacles_to_remove.append(obs_id)
        
#         for obs_id in obstacles_to_remove:
#             del obstacles[obs_id]

#         # Plot current obstacles
#         for obs_id, data in obstacles.items():
#             x, y, color = data['x'], data['y'], data['color']
#             if color == 1:
#                 ax.plot(y, x, 'yo', label='Yellow Cone')
#             elif color == 2:
#                 ax.plot(y, x, 'bo', label='Blue Cone')

#     ax.legend()
#     return []

# def yolo_callback(msg):
#     global yolo_bboxes
#     yolo_bboxes.clear()

#     Cone_Sign_states = {
#         "cone_blue": 2,  # 2 represents blue
#         "cone_red": 1  # 1 represents yellow
#     }
    
#     global Cone_Sign

#     for channel in msg.channels:
#         if len(channel.values) >= 5:  # Ensure there is sufficient data
#             bbox = {
#                 'x1': channel.values[1],
#                 'y1': channel.values[2],
#                 'x2': channel.values[3],
#                 'y2': channel.values[4],
#                 'confidence': channel.values[0],
#                 'label': channel.name
#             }
#             yolo_bboxes.append(bbox)
#             rospy.loginfo(f"Processed bounding box for {channel.name}")

#             if channel.name in Cone_Sign_states:
#                 Cone_Sign = Cone_Sign_states[channel.name]

# def obstacle_callback(msg):
#     global yolo_bboxes
#     current_time = time.time()

#     for bbox in yolo_bboxes:
#         if bbox['x1'] <= msg.data[3] <= bbox['x2'] and bbox['y1'] <= msg.data[4] <= bbox['y2']:
#             print(f"Obstacle within bounding box: Label = {bbox['label']}, Confidence = {bbox['confidence']}")
#             print(f"Obstacle Coordinates: x = {msg.data[0]}, y = {msg.data[1]}")
#             print(f"Distance: {msg.data[5]} meters")

#             # Determine the color based on the type of cone
#             color = 0  # Default color if not a known cone type
#             if bbox['label'] == "cone_blue":
#                 color = 2  # Blue
#             elif bbox['label'] == "cone_red":
#                 color = 1  # Yellow

#             # Create a unique obstacle ID
#             obs_id = (msg.data[0], msg.data[1], color)

#             # Add or update the obstacle in the dictionary
#             with obstacles_lock:
#                 obstacles[obs_id] = {
#                     'x': msg.data[0],
#                     'y': msg.data[1],
#                     'color': color,
#                     'timestamp': current_time
#                 }

# def listener():
#     rospy.init_node('object_detection_listener', anonymous=True)
#     rospy.Subscriber('/yolov9_detect', PointCloud, yolo_callback)
#     rospy.Subscriber('/obstacle_info', Float32MultiArray, obstacle_callback)

#     ani = FuncAnimation(fig, update, init_func=init, blit=False, interval=50, cache_frame_data=False)
#     plt.show()

#     rospy.spin()

# if __name__ == '__main__':
#     listener()

#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

# Define global variables
Cone_Sign = None
yolo_bboxes = []
obstacles = {}
obstacles_lock = threading.Lock()

# Initialize plot and plot elements
fig, ax = plt.subplots()

# Timeout duration in seconds for considering an obstacle as outdated
timeout_duration = 0.1

# Threshold for considering two positions as the same obstacle (in meters)
position_threshold = 0.1

def init():
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.grid(True)
    ax.set_title('Obstacle Positions')
    ax.set_xlabel('Y')
    ax.set_ylabel('X')
    ax.invert_xaxis()
    ax.plot(0, 0, 'ro', label='LiDAR Position')
    ax.legend()
    return []

def update(frame):
    current_time = time.time()
    ax.clear()
    init()

    # Lock the obstacles dictionary during update
    with obstacles_lock:
        # Remove outdated obstacles
        obstacles_to_remove = []
        for obs_id, data in obstacles.items():
            if current_time - data['timestamp'] > timeout_duration:
                obstacles_to_remove.append(obs_id)
        
        for obs_id in obstacles_to_remove:
            del obstacles[obs_id]

        # Plot current obstacles
        for obs_id, data in obstacles.items():
            x, y, color = data['x'], data['y'], data['color']
            if color == 1:
                ax.plot(y, x, 'yo', label='Yellow Cone')
            elif color == 2:
                ax.plot(y, x, 'bo', label='Blue Cone')

    ax.legend()
    return []

def find_existing_obstacle(x, y, color):
    """Find an existing obstacle that is within the position threshold."""
    for obs_id, data in obstacles.items():
        if abs(data['x'] - x) < position_threshold and abs(data['y'] - y) < position_threshold and data['color'] == color:
            return obs_id
    return None

def yolo_callback(msg):
    global yolo_bboxes
    yolo_bboxes.clear()

    Cone_Sign_states = {
        "cone_blue": 2,  # 2 represents blue
        "cone_red": 1  # 1 represents yellow
    }
    
    global Cone_Sign

    for channel in msg.channels:
        if len(channel.values) >= 5:  # Ensure there is sufficient data
            bbox = {
                'x1': channel.values[1],
                'y1': channel.values[2],
                'x2': channel.values[3],
                'y2': channel.values[4],
                'confidence': channel.values[0],
                'label': channel.name
            }
            yolo_bboxes.append(bbox)
            rospy.loginfo(f"Processed bounding box for {channel.name}")

            if channel.name in Cone_Sign_states:
                Cone_Sign = Cone_Sign_states[channel.name]

def obstacle_callback(msg):
    global yolo_bboxes
    current_time = time.time()

    for bbox in yolo_bboxes:
        if bbox['x1'] <= msg.data[3] <= bbox['x2'] and bbox['y1'] <= msg.data[4] <= bbox['y2']:
            print(f"Obstacle within bounding box: Label = {bbox['label']}, Confidence = {bbox['confidence']}")
            print(f"Obstacle Coordinates: x = {msg.data[0]}, y = {msg.data[1]}")
            print(f"Distance: {msg.data[5]} meters")

            # Determine the color based on the type of cone
            color = 0  # Default color if not a known cone type
            if bbox['label'] == "cone_blue":
                color = 2  # Blue
            elif bbox['label'] == "cone_red":
                color = 1  # Yellow

            # Try to find an existing obstacle within the position threshold
            obs_id = find_existing_obstacle(msg.data[0], msg.data[1], color)

            if obs_id is None:
                # Create a new obstacle ID if not found
                obs_id = (msg.data[0], msg.data[1], color)

            # Add or update the obstacle in the dictionary
            with obstacles_lock:
                obstacles[obs_id] = {
                    'x': msg.data[0],
                    'y': msg.data[1],
                    'color': color,
                    'timestamp': current_time
                }

def listener():
    rospy.init_node('object_detection_listener', anonymous=True)
    rospy.Subscriber('/yolov9_detect', PointCloud, yolo_callback)
    rospy.Subscriber('/obstacle_info', Float32MultiArray, obstacle_callback)

    ani = FuncAnimation(fig, update, init_func=init, blit=False, interval=50, cache_frame_data=False)
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    listener()
