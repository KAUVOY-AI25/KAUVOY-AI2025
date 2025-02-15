#!/usr/bin/env python3

import rospy
from erp_driver.msg import erpStatusMsg
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Point, Pose
import time
import math

class EncoderIMUOdometry:
    def __init__(self):
        rospy.init_node('encoder_imu_odometry', anonymous=True)

        # Subscribers
        #rospy.Subscriber('/synchronized_encoder', Int32, self.encoder_callback)
        #rospy.Subscriber('/synchronized_imu', Imu, self.imu_callback)
        #rospy.Subscriber('/erp42/status', erpStatusMsg, self.status_callback)
        rospy.Subscriber('/erp42_encoder', Int32, self.encoder_callback)  #test
        rospy.Subscriber('/vectornav/IMU', Imu, self.imu_callback) #test

        # Publisher
        self.odom_pub = rospy.Publisher('/odometry', Odometry, queue_size=10)

        # Wheel & Encoder Parameters
        self.wheel_diameter_m = 13 * 2 * 0.0254  # 13인치 * 2 * 변환 (단위: 미터)
        self.wheel_circumference = math.pi * self.wheel_diameter_m
        self.encoder_resolution = 120
        self.distance_per_pulse = self.wheel_circumference / self.encoder_resolution

        # Variables
        self.previous_pulse = 0
        self.previous_time = rospy.Time.now().to_sec()
        self.x = 0.0  # 위치 X (m)
        self.y = 0.0  # 위치 Y (m)
        self.yaw = 0.0  # 방향 (라디안)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.angular_velocity_z = 0.0
        self.orientation_q = Quaternion()
        self.is_first_callback = True

    def encoder_callback(self, msg):
        current_pulse = msg.data
        current_time = rospy.Time.now().to_sec()
        
        # 첫 콜백이면 초기화 후 리턴
        if self.is_first_callback:
            self.previous_pulse = current_pulse
            self.previous_time = current_time
            self.is_first_callback = False
            return
        
        # 경과 시간 계산
        time_elapsed = current_time - self.previous_time
        if time_elapsed <= 0:
            return
        
        # 이동 거리 계산
        pulse_change = current_pulse - self.previous_pulse
        distance = pulse_change * self.distance_per_pulse

        # 방향(yaw)을 고려한 x, y 위치 업데이트
        self.x += distance * math.cos(self.yaw)
        self.y += distance * math.sin(self.yaw)

        # 속도 계산
        self.vx = distance / time_elapsed * math.cos(self.yaw)
        self.vy = distance / time_elapsed * math.sin(self.yaw)
        self.vz = 0.0

        # 이전 값 업데이트
        self.previous_pulse = current_pulse
        self.previous_time = current_time

        rospy.loginfo(f"Encoder Data -> X: {self.x:.4f}, Y: {self.y:.4f}, vx: {self.vx:.4f}, vy: {self.vy:.4f}")
        self.publish_odometry()

    def imu_callback(self, msg):
        self.orientation_q = msg.orientation
        self.angular_velocity_z = msg.angular_velocity.z
        
        # 쿼터니언에서 yaw 값 계산
        self.yaw = self.quaternion_to_yaw(
            self.orientation_q.x, self.orientation_q.y,
            self.orientation_q.z, self.orientation_q.w)
        
        rospy.loginfo(f"IMU Data -> Yaw: {self.yaw:.4f} rad, Angular Velocity Z: {self.angular_velocity_z:.4f}")

    def status_callback(self, msg):
        rospy.loginfo(f"ERP42 Status Received: {msg}")

    @staticmethod
    def quaternion_to_yaw(qx, qy, qz, qw):
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy**2 + qz**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 위치 정보 설정
        odom.pose.pose.position = Point(self.x, self.y, 0.0)
        odom.pose.pose.orientation = self.orientation_q

        # 속도 정보 설정
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz
        odom.twist.twist.angular.z = self.angular_velocity_z

        rospy.loginfo(f"Publishing Odometry -> X: {self.x:.4f}, Y: {self.y:.4f}, Orientation: {self.orientation_q}")
        
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        odometry_node = EncoderIMUOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass