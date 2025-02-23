# ================================================================================
# Main Author: 이창현
# Recently Modified Date: 2025-02-16 (V2.0)
# Dependency: ublox_gps/fix
# Description: GPS 정보 수신 후 WGS84(1), UTM(2) 의 2개의 WAYPOINT.txt 생성
#             -> 5m 간격으로 waypoint 생성, KML 파일 생성까지
# ================================================================================

import rospy
import pyproj
import math
import time
from sensor_msgs.msg import NavSatFix
import datetime
import simplekml

# 사용자 지정 장소 설정
location = "kau"  # 원하는 장소명으로 변경 가능

# 현재 날짜 가져오기 (YYYYMMDD 형식)
current_date = datetime.datetime.now().strftime("%m%d_%H:%M")

# 파일 이름 설정
wgs84_filename = f"{current_date}_{location}_WGS84.txt"
utm_filename = f"{current_date}_{location}_UTM.txt"
kml_filename = f"{current_date}_{location}_waypoints.kml"

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
    
    if distance >= 0.5:
        last_x, last_y = x, y
        last_saved_time = current_time
        return True
    return False

def callback(data):
    lat, lon = data.latitude, data.longitude
    x, y = wgs84_to_utm(lat, lon)
    
    if should_save_waypoint(x, y):
        # WGS84 저장
        with open(wgs84_filename, "a") as wgs_file:
            wgs_file.write(f"{lat}, {lon}\n")
        
        # UTM 변환 후 저장
        with open(utm_filename, "a") as utm_file:
            utm_file.write(f"{x} {y}\n")

def create_kml(wgs84_filename, kml_filename):
    kml = simplekml.Kml()
    coordinates = []
    
    with open(wgs84_filename, 'r') as file:
        for line in file:
            lat, lon = map(float, line.strip().split(','))
            coordinates.append((lon, lat))
            
            # 개별 점 (Placemark) 추가
            pnt = kml.newpoint()
            pnt.name = "Waypoint"
            pnt.coords = [(lon, lat)]
            pnt.style.iconstyle.icon.href = "http://maps.google.com/mapfiles/kml/paddle/red-circle.png"
    
    # 경로 (LineString) 추가
    if coordinates:
        linestring = kml.newlinestring()
        linestring.coords = coordinates
        linestring.style.linestyle.color = simplekml.Color.red  # 경로 색상
        linestring.style.linestyle.width = 3  # 경로 두께
    
    kml.save(kml_filename)
    print(f"KML 파일이 생성되었습니다: {kml_filename}")

def listener():
    rospy.init_node('waypoint_logger', anonymous=True)
    rospy.Subscriber("/ublox_gps/fix", NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
        create_kml(wgs84_filename, kml_filename)
    except rospy.ROSInterruptException:
        pass

