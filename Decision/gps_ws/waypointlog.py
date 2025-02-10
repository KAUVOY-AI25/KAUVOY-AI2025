# ================================================================================
# Main Author: 이창현
# Recently Modified Date: 2025-02-09 (V1.0)
# Dependency: ublox_gps/fix
# Description: GPS 정보 수신 후 WGS84(1), UTM(2) 의 2개의 WAYPOINT.txt 생성
#             -> 1m 간격으로 waypoint 생성, but 1m 안되더라도 시간 3초 지나면 waypoint생성 
# ================================================================================

import rospy
import pyproj
import math
import time
from sensor_msgs.msg import NavSatFix

last_saved_time = 0
last_x, last_y = None, None
def wgs84_to_utm(lat, lon):
    proj = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', south=False)
    x, y = proj(lon, lat)
    return x, y

def should_save_waypoint(x, y):
    global last_saved_time, last_x, last_y
    current_time = time.time()
    
    if last_x is None or last_y is None:
        last_x, last_y = x, y
        last_saved_time = current_time
        return True
    
    distance = math.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)
    time_elapsed = current_time - last_saved_time
    
    if distance >= 1.0 and time_elapsed >= 3.0:
        last_x, last_y = x, y
        last_saved_time = current_time
        return True
    return False

def callback(data):
    lat, lon = data.latitude, data.longitude
    x, y = wgs84_to_utm(lat, lon)
    
    if should_save_waypoint(x, y):
        # WGS84 저장
        with open("waypoint_wgs84.txt", "a") as wgs_file:
            wgs_file.write(f"{lat}, {lon}\n")
        
        # UTM 변환 후 저장
        with open("250210_kau.txt", "a") as utm_file:
            utm_file.write(f"{x} {y}\n")

def listener():
    rospy.init_node('waypoint_logger', anonymous=True)
    rospy.Subscriber("/ublox_gps/fix", NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

