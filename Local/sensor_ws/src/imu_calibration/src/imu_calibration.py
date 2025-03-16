#!/usr/bin/env python
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
        """ 자기장 센서 데이터를 이용해 자북 방향(Yaw) 계산 """
        mag_x = msg.magnetic_field.x
        mag_y = msg.magnetic_field.y

        # atan2를 이용한 자북 방향 계산 (라디안 단위)
        self.mag_yaw = math.atan2(mag_y, mag_x)

    def imu_callback(self, msg):
        """ IMU 데이터를 보정해서 자북 기준으로 변환 """
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

        # 보정된 IMU 데이터 생성
        corrected_imu = Imu()
        corrected_imu.header = Header()
        corrected_imu.header.stamp = rospy.Time.now()
        corrected_imu.header.frame_id = "base_link"
        corrected_imu.orientation.x = q_corrected[0]
        corrected_imu.orientation.y = q_corrected[1]
        corrected_imu.orientation.z = q_corrected[2]
        corrected_imu.orientation.w = q_corrected[3]

        # 보정된 IMU 데이터 퍼블리시
        self.imu_pub.publish(corrected_imu)

if __name__ == "__main__":
    try:
        IMUCalibration()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

