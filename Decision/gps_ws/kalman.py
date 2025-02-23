#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu

class KalmanFilter:
    def __init__(self):
        # 상태 변수 (예: 각속도, 가속도 등)
        self.x = np.zeros(6)  # [roll, pitch, yaw, v_roll, v_pitch, v_yaw]

        # 상태 전이 행렬
        self.F = np.eye(6)
        self.F[0, 3] = 1  # roll과 roll 속도의 관계
        self.F[1, 4] = 1  # pitch와 pitch 속도의 관계
        self.F[2, 5] = 1  # yaw와 yaw 속도의 관계

        # 측정 모델 (IMU 데이터는 roll, pitch, yaw만 포함한다고 가정)
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0]])

        # 측정 노이즈 공분산 (임의로 설정)
        self.R = np.eye(3) * 0.1

        # 상태 오차 공분산 (임의로 설정)
        self.P = np.eye(6)

        # 프로세스 노이즈 공분산 (임의로 설정)
        self.Q = np.eye(6) * 0.01

    def predict(self, dt):
        # 상태 전이 행렬 업데이트 (시간 변화 반영)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        # 상태 예측
        self.x = self.F @ self.x

        # 오차 공분산 예측
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        # 칼만 이득 계산
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 상태 업데이트
        y = z - (self.H @ self.x)
        self.x = self.x + (K @ y)

        # 오차 공분산 업데이트
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

# ROS 노드
class IMUKalmanFilterNode:
    def __init__(self):
        rospy.init_node('imu_kalman_filter_node', anonymous=True)

        # Kalman Filter 객체 생성
        self.kf = KalmanFilter()

        # IMU 데이터를 구독
        rospy.Subscriber("/vectornav/IMU", Imu, self.imu_callback)

        # 필터링된 IMU 데이터를 퍼블리시할 Publisher
        self.pub = rospy.Publisher("/EKF_IMU", Imu, queue_size=10)

        # 루프 속도 설정
        self.rate = rospy.Rate(10)  # 10Hz

    def imu_callback(self, msg):
        # IMU 데이터에서 roll, pitch, yaw 추출
        roll = msg.orientation.x
        pitch = msg.orientation.y
        yaw = msg.orientation.z
        rospy.loginfo("RAW_Estimated roll: %f, pitch: %f, yaw: %f", self.kf.x[0], self.kf.x[1], self.kf.x[2])

        # 측정값 벡터
        z = np.array([roll, pitch, yaw])

        # 시간 간격 계산 (예시로 0.1초 설정)
        dt = 0.1

        # 칼만 필터 예측 단계
        self.kf.predict(dt)

        # 칼만 필터 업데이트 단계
        self.kf.update(z)

        # 추정된 상태 출력 (필요시 사용 가능)
        rospy.loginfo("EKF_Estimated roll: %f, pitch: %f, yaw: %f", self.kf.x[0], self.kf.x[1], self.kf.x[2])

        # 필터링된 IMU 데이터 퍼블리시
        filtered_imu_msg = Imu()
        filtered_imu_msg.header.stamp = rospy.Time.now()
        filtered_imu_msg.header.frame_id = msg.header.frame_id

        # 필터링된 roll, pitch, yaw을 orientation에 적용
        filtered_imu_msg.orientation.x = self.kf.x[0]  # 필터링된 roll
        filtered_imu_msg.orientation.y = self.kf.x[1]  # 필터링된 pitch
        filtered_imu_msg.orientation.z = self.kf.x[2]  # 필터링된 yaw
        filtered_imu_msg.orientation.w = msg.orientation.w  # w 값은 그대로 유지

        # IMU의 다른 값들도 그대로 유지
        filtered_imu_msg.angular_velocity = msg.angular_velocity
        filtered_imu_msg.linear_acceleration = msg.linear_acceleration

        # 필터링된 IMU 데이터를 퍼블리시
        self.pub.publish(filtered_imu_msg)

    def spin(self):
        while not rospy.is_shutdown():
            # 10Hz 속도로 루프 실행
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = IMUKalmanFilterNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
