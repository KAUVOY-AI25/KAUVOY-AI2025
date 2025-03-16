#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point

class EncoderIMUOdometry:
    def __init__(self):
        rospy.init_node('odom_encoder', anonymous=True)

        # Subscribers
        rospy.Subscriber('/erp42_encoder', Int32, self.encoder_callback)
        rospy.Subscriber('/vectornav/IMU', Imu, self.imu_callback) 

        # Publisher
        self.odom_pub = rospy.Publisher('/odometry', Odometry, queue_size=10)

        # Wheel & Encoder Parameters
        self.wheel_diameter_m = 13 * 2 * 0.0254  # 13인치 * 2 * 변환 (미터)
        self.wheel_circumference = math.pi * self.wheel_diameter_m
        self.encoder_resolution = 120
        self.distance_per_pulse = self.wheel_circumference / self.encoder_resolution

        # Variables
        self.previous_pulse = 0
        self.previous_time = rospy.Time.now().to_sec()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.angular_velocity_z = 0.0
        self.orientation_q = Quaternion()
        self.is_first_callback = True

        # IMU 데이터 수신 체크
        self.last_imu_time = rospy.Time.now().to_sec()
        self.imu_timeout = 0.5  # 0.5초 이상 IMU 데이터 수신 안 되면 yaw 보정

        # ROS Loop Rate
        self.rate = rospy.Rate(50)  # 50Hz (0.02초 주기)

    def encoder_callback(self, msg):
        current_pulse = msg.data
        current_time = rospy.Time.now().to_sec()
        
        if self.is_first_callback:
            self.previous_pulse = current_pulse
            self.previous_time = current_time
            self.is_first_callback = False
            return
        
        time_elapsed = current_time - self.previous_time
        if time_elapsed <= 0:
            return
        
        pulse_change = current_pulse - self.previous_pulse
        distance = pulse_change * self.distance_per_pulse

        # IMU 데이터 유실 감지 (yaw 업데이트가 중단된 경우)
        if current_time - self.last_imu_time > self.imu_timeout:
            rospy.logwarn("IMU data timeout! Estimating yaw using angular velocity.")
            self.yaw += self.angular_velocity_z * time_elapsed  # yaw 보정

        # 방향(yaw)을 고려한 x, y 위치 업데이트
        self.x += distance * math.cos(self.yaw)
        self.y += distance * math.sin(self.yaw)

        # 속도 계산
        self.vx = distance / time_elapsed * math.cos(self.yaw)
        self.vy = distance / time_elapsed * math.sin(self.yaw)

        self.previous_pulse = current_pulse
        self.previous_time = current_time

        self.publish_odometry()

    def imu_callback(self, msg):
        self.orientation_q = msg.orientation
        self.angular_velocity_z = msg.angular_velocity.z
        new_yaw = self.quaternion_to_yaw(
            self.orientation_q.x, self.orientation_q.y,
            self.orientation_q.z, self.orientation_q.w)
        
        # yaw 업데이트 (부드럽게 보정)
        alpha = 0.9  # 필터 계수 (0에 가까울수록 천천히 변경됨)
        self.yaw = 0 #alpha * new_yaw + (1 - alpha) * self.yaw  

        self.last_imu_time = rospy.Time.now().to_sec()  # 마지막 IMU 수신 시간 기록

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

        odom.pose.pose.position = Point(self.x, self.y, 0.0)
        odom.pose.pose.orientation = self.orientation_q

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = 0 #self.angular_velocity_z

        self.odom_pub.publish(odom)

        # 🚀 터미널에서 실시간 Odometry 확인 (로그 출력)
        rospy.loginfo(
            "Odometry -> x: {:.3f}, y: {:.3f}, yaw: {:.3f} rad, vx: {:.2f} m/s, vy: {:.2f} m/s, ω: {:.3f} rad/s".format(
                self.x, self.y, self.yaw, self.vx, self.vy, self.angular_velocity_z
            )
        )

    def run(self):
        """ ROS 노드 실행을 위한 루프 """
        while not rospy.is_shutdown():
            self.publish_odometry()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = EncoderIMUOdometry()
        node.run()
    except rospy.ROSInterruptException:
        pass
