#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Int32

# 퍼블리셔 생성
gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
imu_pub = rospy.Publisher('/synchronized_imu', Imu, queue_size=10)
encoder_pub = rospy.Publisher('/synchronized_encoder', Int32, queue_size=10)

# 동기화된 데이터 콜백
def sync_callback(gps_data, imu_data, encoder_data):
    # 동기화된 GPS 데이터 퍼블리시
    gps_pub.publish(gps_data)
    
    # GPS 데이터 출력
    rospy.loginfo("GPS Data: Latitude: %.6f, Longitude: %.6f, Altitude: %.2f", 
                 gps_data.latitude, gps_data.longitude, gps_data.altitude)
    
    # IMU 데이터 전체 출력
    rospy.loginfo("IMU Data: ")
    rospy.loginfo("Orientation: (x: %.3f, y: %.3f, z: %.3f, w: %.3f)", 
                 imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w)
    rospy.loginfo("Angular Velocity: (x: %.3f, y: %.3f, z: %.3f)", 
                 imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z)
    rospy.loginfo("Linear Acceleration: (x: %.3f, y: %.3f, z: %.3f)", 
                 imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z)
    
    # Encoder 데이터 출력
    rospy.loginfo("Encoder Data: %d", encoder_data.data)

    # 동기화된 IMU, Encoder 데이터 퍼블리시
    imu_pub.publish(imu_data)
    encoder_pub.publish(encoder_data)

def synchronize_data():
    rospy.init_node('data_sync_node')

    # GPS, IMU, Encoder 데이터 구독
    gps_sub = message_filters.Subscriber('/ublox_gps/fix', NavSatFix)
    imu_sub = message_filters.Subscriber('/vectornav/IMU', Imu) 
    encoder_sub = message_filters.Subscriber('/erp42_encoder', Int32)

    # Time synchronizer 설정 (allow_headerless=True)
    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub, encoder_sub], queue_size=30, slop=0.033, allow_headerless=True)
    ts.registerCallback(sync_callback)

if __name__ == '__main__':
    try:
        synchronize_data()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass