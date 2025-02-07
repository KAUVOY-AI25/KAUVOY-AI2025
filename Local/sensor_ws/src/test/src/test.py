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

