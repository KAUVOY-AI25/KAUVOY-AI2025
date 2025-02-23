import rospy
import matplotlib.pyplot as plt
import numpy as np
import tf.transformations as tf_trans
from nav_msgs.msg import Odometry

# 데이터 저장용 리스트
time_stamps = []
heading_angles = []

def odometry_callback(msg):
    global time_stamps, heading_angles

    # Quaternion -> Roll, Pitch, Yaw 변환
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    _, _, yaw = tf_trans.euler_from_quaternion(quaternion)

    # rad -> deg 변환
    heading_angle = np.degrees(yaw)

    # 시간 & 헤딩 각도 저장
    time_stamps.append(rospy.get_time())
    heading_angles.append(heading_angle)

    # 그래프 업데이트
    plt.clf()
    plt.plot(time_stamps, heading_angles, label="Heading Angle (deg)", color="blue")
    plt.xlabel("Time (s)")
    plt.ylabel("Heading Angle (degrees)")
    plt.title("Real-time Heading Angle")
    plt.legend()
    plt.grid(True)

    plt.pause(0.1)

def main():
    rospy.init_node("heading_plotter", anonymous=True)
    rospy.Subscriber("/odometry/filtered_map", Odometry, odometry_callback)

    plt.ion()
    plt.show()

    rospy.spin()

if __name__ == "__main__":
    main()
