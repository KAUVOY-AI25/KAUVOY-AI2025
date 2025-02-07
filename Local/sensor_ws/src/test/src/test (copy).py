#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

class EncoderYawSmoothSimulator:
    def __init__(self):
        # Node 초기화
        rospy.init_node("encoder_yaw_smooth_simulator", anonymous=True)

        # Publisher 초기화
        self.encoder_pub = rospy.Publisher("/encoder_distance", Float64, queue_size=10)
        self.yaw_pub = rospy.Publisher("/calculated_yaw", Float64, queue_size=10)

        # Encoder 거리값
        self.encoder_distance = 0.0

        # Yaw 값 관련 변수
        self.yaw = 0.0
        self.state = "increase_to_90"  # 초기 상태: 90°까지 증가
        self.start_time = rospy.get_time()  # 상태 변경 기준 시간

        # Timer를 사용하여 Encoder 값과 Yaw 값 퍼블리시
        rospy.Timer(rospy.Duration(0.1), self.publish_values)

    def publish_values(self, event):
        """Encoder 값과 Yaw 값을 퍼블리시"""
        # Encoder 값 증가
        self.encoder_distance += 1.0
        self.encoder_pub.publish(self.encoder_distance)

        # 상태에 따른 Yaw 값 점진적 변경
        current_time = rospy.get_time()
        elapsed_time = current_time - self.start_time

        if elapsed_time >= 3.0:
            # 3초가 경과하면 다음 상태로 전환
            self.start_time = current_time
            if self.state == "increase_to_90":
                self.state = "decrease_to_0"
            elif self.state == "decrease_to_0":
                self.state = "decrease_to_minus_90"
            elif self.state == "decrease_to_minus_90":
                self.state = "increase_to_0"
            elif self.state == "increase_to_0":
                self.state = "increase_to_90"

        # 상태에 따라 Yaw 값을 계산
        if self.state == "increase_to_90":
            self.yaw = 90.0 * (elapsed_time / 3.0)  # 0 -> 90
        elif self.state == "decrease_to_0":
            self.yaw = 90.0 * (1 - (elapsed_time / 3.0))  # 90 -> 0
        elif self.state == "decrease_to_minus_90":
            self.yaw = -90.0 * (elapsed_time / 3.0)  # 0 -> -90
        elif self.state == "increase_to_0":
            self.yaw = -90.0 * (1 - (elapsed_time / 3.0))  # -90 -> 0

        # Yaw 값 퍼블리시
        self.yaw_pub.publish(self.yaw)

        # 현재 상태와 값을 로그로 출력
        rospy.loginfo(f"State: {self.state}, Yaw: {self.yaw:.2f}°, Encoder Distance: {self.encoder_distance}")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = EncoderYawSmoothSimulator()
        node.run()
    except rospy.ROSInterruptException:
        pass

