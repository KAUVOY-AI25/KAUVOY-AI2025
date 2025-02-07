#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import random

def imu_callback(msg):
    # 수신한 IMU 데이터를 수정
    modified_msg = Imu()
    modified_msg.header = msg.header  # 기존 헤더 유지

    # 가속도 값 임의 변경 (-0.5 ~ 0.5 추가)
    modified_msg.linear_acceleration.x = msg.linear_acceleration.x + random.uniform(-0.5, 0.5)
    modified_msg.linear_acceleration.y = msg.linear_acceleration.y + random.uniform(-0.5, 0.5)
    modified_msg.linear_acceleration.z = msg.linear_acceleration.z + random.uniform(-0.5, 0.5)

    # 각속도 값 임의 변경 (-0.1 ~ 0.1 추가)
    modified_msg.angular_velocity.x = msg.angular_velocity.x + random.uniform(-0.1, 0.1)
    modified_msg.angular_velocity.y = msg.angular_velocity.y + random.uniform(-0.1, 0.1)
    modified_msg.angular_velocity.z = msg.angular_velocity.z + random.uniform(-0.1, 0.1)

    # 방향(Quaternion)은 그대로 사용
    modified_msg.orientation = msg.orientation

    # 수정된 데이터를 발행
    imu_pub.publish(modified_msg)

    rospy.loginfo(f"[Modified IMU] Linear Acc: [{modified_msg.linear_acceleration.x:.2f}, {modified_msg.linear_acceleration.y:.2f}, {modified_msg.linear_acceleration.z:.2f}] "
                  f"Angular Vel: [{modified_msg.angular_velocity.x:.2f}, {modified_msg.angular_velocity.y:.2f}, {modified_msg.angular_velocity.z:.2f}]")

def imu_modifier():
    global imu_pub
    rospy.init_node('imu_modifier', anonymous=True)

    # 수정된 IMU 데이터를 발행할 퍼블리셔
    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)

    # 원본 IMU 데이터를 구독
    rospy.Subscriber('/vectornav/IMU', Imu, imu_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        imu_modifier()
    except rospy.ROSInterruptException:
        pass

