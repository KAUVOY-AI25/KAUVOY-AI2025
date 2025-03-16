#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import random

def encoder_publisher():
    rospy.init_node('encoder_publisher', anonymous=True)
    pub = rospy.Publisher('/erp42_encoder', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz (0.1초 간격)

    encoder_value = 0
    target_value = random.randint(1, 260)  # 첫 랜덤 목표값
    increment = True  # 증가/감소 방향 플래그

    while not rospy.is_shutdown():
        # 현재 값이 목표값에 도달하면 새로운 목표값 생성
        if increment and encoder_value >= target_value:
            increment = False  # 감소 방향으로 전환
            target_value = random.randint(1, 260)  # 새로운 목표값 생성
            rospy.loginfo(f"Switching to decrement, new target: {target_value}")
        elif not increment and encoder_value <= -target_value:
            increment = True  # 증가 방향으로 전환
            target_value = random.randint(1, 260)  # 새로운 목표값 생성
            rospy.loginfo(f"Switching to increment, new target: {target_value}")

        # 증가 또는 감소 수행
        if increment:
            encoder_value += 1
        else:
            encoder_value -= 1

        # 메시지 발행
        msg = Int32()
        msg.data = encoder_value
        pub.publish(msg)

        rospy.loginfo(f"Encoder value: {encoder_value}, Target: {target_value}")

        rate.sleep()

if __name__ == '__main__':
    try:
        encoder_publisher()
    except rospy.ROSInterruptException:
        pass
"""
import rospy
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import math
from scipy.spatial.transform import Rotation as R

class IMUCalibration:
    def __init__(self):
        rospy.init_node("imu_calibration")

        # IMU 데이터 & 자기장 데이터 구독
        rospy.Subscriber("/vectornav/IMU", Imu, self.imu_callback)
        rospy.Subscriber("/vectornav/Mag", MagneticField, self.mag_callback)

        # 보정된 IMU 데이터 퍼블리셔
        self.imu_pub = rospy.Publisher("/vectornav/IMU_calibrated", Imu, queue_size=10)

        # 변수 초기화
        self.mag_yaw = None  # 자기장 센서에서 계산된 자북 기준 Yaw
        self.imu_yaw = None  # 원본 IMU Yaw
        self.yaw_offset = None  # 보정해야 할 오프셋 (자북 기준)

    def mag_callback(self, msg):
        #자기장 센서 데이터를 이용해 자북 방향(Yaw) 계산
        mag_x = msg.magnetic_field.x
        mag_y = msg.magnetic_field.y

        # atan2를 이용한 자북 방향 계산 (라디안 단위)
        self.mag_yaw = math.atan2(mag_y, mag_x)

    def imu_callback(self, msg):
        #IMU 데이터를 보정해서 자북 기준으로 변환
        if self.mag_yaw is None:
            return  # 자기장 데이터 없으면 보정 불가

        # 현재 IMU의 오일러 각도 변환
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = R.from_quat(quat).as_euler('xyz', degrees=False)

        self.imu_yaw = yaw  # 원본 Yaw 저장

        # 오프셋 계산 (자북 Yaw - 현재 IMU Yaw)
        if self.yaw_offset is None:
            self.yaw_offset = self.mag_yaw - self.imu_yaw
            rospy.loginfo(f"Yaw Offset (IMU to North): {math.degrees(self.yaw_offset)} degrees")

        # 보정된 Yaw 계산
        corrected_yaw = self.imu_yaw + self.yaw_offset

        # 다시 쿼터니언으로 변환
        q_corrected = R.from_euler('xyz', [roll, pitch, corrected_yaw], degrees=False).as_quat()

        # 보정된 IMU 데이터 생성 (기존 데이터 유지)
        corrected_imu = Imu()
        corrected_imu.header = msg.header  # 원본 헤더 유지
        corrected_imu.orientation.x = q_corrected[0]
        corrected_imu.orientation.y = q_corrected[1]
        corrected_imu.orientation.z = q_corrected[2]
        corrected_imu.orientation.w = q_corrected[3]
        corrected_imu.orientation_covariance = msg.orientation_covariance
        corrected_imu.angular_velocity = msg.angular_velocity  # 각속도 유지
        corrected_imu.angular_velocity_covariance = msg.angular_velocity_covariance
        corrected_imu.linear_acceleration = msg.linear_acceleration  # 선가속도 유지
        corrected_imu.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # 보정된 IMU 데이터 퍼블리시
        self.imu_pub.publish(corrected_imu)

if __name__ == "__main__":
    try:
        IMUCalibration()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
"""
