#!/usr/bin/env python3

import rospy
from erp_driver.msg import erpStatusMsg
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import time
import math

class EncoderIMUOdometry:
    def __init__(self):
        rospy.init_node('encoder_imu_odometry', anonymous=True)
        
        # Subscribers
        rospy.Subscriber('/synchronized_encoder', Int32, self.encoder_callback)
        rospy.Subscriber('/synchronized_imu', Imu, self.imu_callback)
        rospy.Subscriber('/erp42/status', erpStatusMsg, self.status_callback)
        
        # Publisher
        self.odom_pub = rospy.Publisher('/odometry', Odometry, queue_size=10)
        
        # Wheel & Encoder Parameters
        self.wheel_diameter_m = 13 * 2 * 0.0254  # 13인치 타이어 * 2(지름) * m/인치 변환
        self.wheel_circumference = math.pi * self.wheel_diameter_m
        self.encoder_resolution = 120
        self.distance_per_pulse = self.wheel_circumference / self.encoder_resolution
        
        # Variables
        self.previous_pulse = 0
        self.previous_time = time.time()
        self.total_distance = 0.0

        # 로컬 좌표계에서의 추정 위치
        self.x = 0.0
        self.y = 0.0

        # 속도 및 자세
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw = 0.0
        self.angular_velocity_z = 0.0
        self.orientation_q = Quaternion()

        # 콜백 최초 호출 여부
        self.is_first_callback = True

    def encoder_callback(self, msg):
        current_pulse = msg.data
        
        # 첫 콜백이면 초기화 후 리턴
        if self.is_first_callback:
            self.previous_pulse = current_pulse
            self.previous_time = time.time()
            self.is_first_callback = False
            return
        
        # 시간 계산
        current_time = time.time()
        time_elapsed = current_time - self.previous_time
        
        # 엔코더 펄스 변화량 → 이동 거리
        pulse_change = current_pulse - self.previous_pulse
        distance = pulse_change * self.distance_per_pulse
        self.total_distance += distance
        
        if time_elapsed > 0:
            # 로봇의 선속도 계산
            speed = distance / time_elapsed
            self.vx = speed * math.cos(self.yaw)
            self.vy = speed * math.sin(self.yaw)
            self.vz = 0.0

            # (x, y) 위치 적분
            self.x += distance * math.cos(self.yaw)
            self.y += distance * math.sin(self.yaw)
        
        # 다음 루프를 위한 값 업데이트
        self.previous_pulse = current_pulse
        self.previous_time = current_time
        
        rospy.loginfo(
            f"encoder: {current_pulse} | "
            f"dist: {distance:.4f} | total_distance: {self.total_distance:.4f} | "
            f"vx: {self.vx:.4f}, vy: {self.vy:.4f} | x: {self.x:.4f}, y: {self.y:.4f}"
        )

        self.publish_odometry()
    
    def imu_callback(self, msg):
        # IMU로부터 쿼터니언(orientation)과 z축 각속도
        self.orientation_q = msg.orientation
        self.angular_velocity_z = msg.angular_velocity.z
        
        # 쿼터니언 → yaw 변환
        self.yaw = self.quaternion_to_yaw(
            self.orientation_q.x,
            self.orientation_q.y,
            self.orientation_q.z,
            self.orientation_q.w
        )
        rospy.loginfo(f"Updated Yaw: {self.yaw:.4f} rad, Angular Velocity Z: {self.angular_velocity_z:.4f}")
    
    def status_callback(self, msg):
        rospy.loginfo(f"Status Message Received: {msg}")
    
    @staticmethod
    def quaternion_to_yaw(qx, qy, qz, qw):
        """쿼터니언(qx, qy, qz, qw) → yaw(라디안) 변환"""
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy**2 + qz**2)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def publish_odometry(self):
        # Odometry 메시지 생성
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # 위치 (x, y) 및 자세
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.orientation_q
        
        # 속도 (vx, vy) 및 z축 각속도
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz
        odom.twist.twist.angular.z = self.angular_velocity_z
        
        rospy.loginfo(
            f"Publishing Odometry -> x: {self.x:.4f}, y: {self.y:.4f}, "
            f"Orientation: {self.orientation_q}, Angular Velocity Z: {self.angular_velocity_z:.4f}"
        )
        
        # 퍼블리시
        self.odom_pub.publish(odom)
    
if __name__ == '__main__':
    try:
        odometry_node = EncoderIMUOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
