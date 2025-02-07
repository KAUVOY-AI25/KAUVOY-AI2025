#!/usr/bin/env python3

import rospy
from erp_driver.msg import erpStatusMsg
from std_msgs.msg import Int32, Float32
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
        self.wheel_diameter_m = 13 * 2 * 0.0254  # 13인치 * 2 * 변환
        self.wheel_circumference = math.pi * self.wheel_diameter_m
        self.encoder_resolution = 120
        self.distance_per_pulse = self.wheel_circumference / self.encoder_resolution
        
        # Variables
        self.previous_pulse = 0
        self.previous_time = time.time()
        self.total_distance = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw = 0.0
        self.angular_velocity_z = 0.0
        self.orientation_q = Quaternion()
        self.is_first_callback = True

    def encoder_callback(self, msg):
        current_pulse = msg.data
        
        # 첫 콜백이면 초기화
        if self.is_first_callback:
            self.previous_pulse = current_pulse
            self.previous_time = time.time()
            self.is_first_callback = False
            return
        
        # 시간 계산
        current_time = time.time()
        time_elapsed = current_time - self.previous_time
        
        # 펄스 변화량 계산
        pulse_change = current_pulse - self.previous_pulse
        distance = pulse_change * self.distance_per_pulse
        self.total_distance += distance
        
        # 속도 계산
        if time_elapsed > 0:
            self.vx = (distance / time_elapsed) * math.cos(self.yaw)
            self.vy = (distance / time_elapsed) * math.sin(self.yaw)
            self.vz = 0.0
        
        # 이전 값 업데이트
        self.previous_pulse = current_pulse
        self.previous_time = current_time
        
        rospy.loginfo(f"encoder: {current_pulse} / total_distance: {self.total_distance:.4f} / vx: {self.vx:.4f} / vy: {self.vy:.4f}")
        self.publish_odometry()
    
    def imu_callback(self, msg):
        self.orientation_q = msg.orientation
        self.angular_velocity_z = msg.angular_velocity.z
        
        # 쿼터니언에서 yaw 값 계산
        self.yaw = self.quaternion_to_yaw(
            self.orientation_q.x, self.orientation_q.y,
            self.orientation_q.z, self.orientation_q.w)
        
        rospy.loginfo(f"Updated Yaw: {self.yaw:.4f} radians, Angular Velocity Z: {self.angular_velocity_z:.4f}")
    
    def status_callback(self, msg):
        rospy.loginfo(f"Status Message Received: {msg}")
    
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
        
        odom.pose.pose.orientation = self.orientation_q
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz
        odom.twist.twist.angular.z = self.angular_velocity_z
        
        rospy.loginfo(f"Publishing Odometry -> Orientation: {self.orientation_q}, Angular Velocity Z: {self.angular_velocity_z:.4f}")
        
        self.odom_pub.publish(odom)
    
if __name__ == '__main__':
    try:
        odometry_node = EncoderIMUOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

