#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Int8, UInt8
from nav_msgs.msg import Odometry
import time

global Traffic_Light, Traffic_Sign, mission_num, mode, status

# Initialize these at the start of your script
Traffic_Light = None
Traffic_Sign = None

# Example initialization to avoid uninitialized access
mission_num = 0
mode = 0
status = 1

def Traffic_Light_callback(msg):
    traffic_light_states = {
        "Red": 0,
        "Yellow": 1,
        "Green": 2,
        "Left_R": 3,
        "Left_G": 4
    }
    
    global Traffic_Light
    
    for channel in msg.channels:
        if channel.name in traffic_light_states:
            Traffic_Light = traffic_light_states[channel.name]
            print(f"Traffic_Light: {Traffic_Light} {channel.name}")
            print("Confidence Score:", channel.values[0])
            print("Bounding Box Coordinates (x1, y1, x2, y2):", channel.values[1:5])

def Traffic_Sign_callback(msg):
    traffic_sign_states = {
        "Intersection": 4,
        "Left": 0,
        "Right": 1,
        "U-Turn": 6,
        "Static-Obs": 2,
        "Dynamic-Obs": 3,
        "Crosswalk": 8,
        "School-Zone": 10,
        "Bump": 7,
        "Parking": 5,
        "Bus": 9
    }

    
    global Traffic_Sign
    
    for channel in msg.channels:
        if channel.name in traffic_sign_states:
            Traffic_Sign = traffic_sign_states[channel.name]
            print(f"Traffic_Sign: {Traffic_Sign} {channel.name}")
            print("Confidence Score:", channel.values[0])
            print("Bounding Box Coordinates (x1, y1, x2, y2):", channel.values[1:5])


# 미션 목록
class MISSION:
   BASE = 0         #0. 일반주행
   TURN_LEFT = 1      #1. 비신호 교차로 좌회전(사)
   TURN_LEFT_INTER = 2    #2. 신호 교차로 좌회전(나)
   FORWARD = 3         #3. 비신호 교차로 직진(가)
   FORWARD_INTER = 4    #4. 신호 교차로 직진(가)
   TURN_RIGHT = 5       #5. 비신호(신호) 교차로 우회전(다, 아)
   STA_OBS = 6         #6. 정적 장애물(라)
   DYN_OBS = 7         #7. 동적 장애물(마)
   PARK = 8          #8. 주차(카)
   UTURN = 9         #9. 유턴(파)
   BUMP = 10         #10. 방지턱(차)
   CROSSWALK = 11      #11. 횡단보도(바)
   BUSONLY = 12         #12. 버스전용(타)

# 표지판 신호
class TRAFFIC_SIGN:
   LEFT_SIGN = 0         #좌회전 표지판
   RIGHT_SIGN = 1      #우회전 표지판
   STA_OBS_SIGN = 2      #공사중(정적 장애물) 표지판
   DYN_OBS_SIGN = 3      #자전거(동적 장애물) 표지판
   INTER_SIGN = 4      #교차로 표지판
   PARK_SIGN = 5      #주차 표지판
   UTURN_SIGN = 6      #유턴 표지판
   BUMP_SIGN = 7      #방지턱 표지판
   CROSSWALK_SIGN = 8   #횡단보도 표지판
   BUSONLY_SIGN = 9      #버스전용차선 표지판

   
def listener():
    rospy.init_node('object_detection_listener', anonymous=True)
    rospy.Subscriber('/yolov9_detect', PointCloud, Traffic_Light_callback)
    rospy.Subscriber('/yolov9_detect', PointCloud, Traffic_Sign_callback)


def Mission_callback(Traffic_Sign, Traffic_Light):
    # global mission_num 
    if Traffic_Sign == TRAFFIC_SIGN.LEFT_SIGN and Traffic_Sign == TRAFFIC_SIGN.INTER_SIGN:
        mission_num = MISSION.TURN_LEFT_INTER
    elif Traffic_Sign == TRAFFIC_SIGN.LEFT_SIGN and Traffic_Sign != TRAFFIC_SIGN.INTER_SIGN:
        mission_num = MISSION.TURN_LEFT
    elif Traffic_Sign == TRAFFIC_SIGN.RIGHT_SIGN:
        mission_num = MISSION.TURN_RIGHT
    elif Traffic_Sign == TRAFFIC_SIGN.STA_OBS_SIGN:
        mission_num = MISSION.STA_OBS
    elif Traffic_Sign == TRAFFIC_SIGN.DYN_OBS_SIGN:
        mission_num = MISSION.DYN_OBS
    elif Traffic_Sign ==  TRAFFIC_SIGN.PARK_SIGN:
        mission_num = MISSION.PARK
    elif Traffic_Sign ==  TRAFFIC_SIGN.UTURN_SIGN:
        mission_num = MISSION.UTURN
    elif Traffic_Sign ==  TRAFFIC_SIGN.BUMP_SIGN:
        mission_num = MISSION.BUMP
    elif Traffic_Sign ==  TRAFFIC_SIGN.CROSSWALK_SIGN:
        mission_num = MISSION.CROSSWALK
    elif Traffic_Sign ==  TRAFFIC_SIGN.BUSONLY_SIGN:
        mission_num = MISSION.BUSONLY
    elif Traffic_Sign == TRAFFIC_SIGN.INTER_SIGN:
        mission_num = MISSION.FORWARD_INTER
    elif Traffic_Sign != TRAFFIC_SIGN.INTER_SIGN and Traffic_Sign != TRAFFIC_SIGN.LEFT_SIGN and Traffic_Sign != TRAFFIC_SIGN.RIGHT_SIGN:
        mission_num = MISSION.FORWARD
    # print("MISSION:", mission_num)
    return mission_num

def Traffic_callback(Traffic_Light):
    # global mission_num, status
    if(Traffic_Light == 2): # GREEN
        status = 1 # race
        mission_num = MISSION.FORWARD_INTER
    
    elif(Traffic_Light == 3): # RED_LEFT
        status.data = 1 # race
        mission_num = MISSION.TURN_LEFT

    elif(Traffic_Light == 4): # GREEN_LEFT
        status.data = 1 # race
        mission_num= MISSION.TURN_LEFT_INTER

    elif(Traffic_Light == 0 or Traffic_Light == 1 or Traffic_Light == 2): # STOP
        status = 0 # stop
    # print("status:", status)
    return status

def Mode_callback(mission):
    if(mission_num == MISSION.BASE or mission_num == MISSION.FORWARD or mission_num == MISSION.FORWARD_INTER):
        mode = 0 # 일반주행       
    elif(mission_num == MISSION.TURN_LEFT or mission_num == MISSION.TURN_LEFT_INTER or mission_num == MISSION.TURN_LEFT_INTER):
        mode = 1 # 좌회전
    elif(mission_num == MISSION.TURN_RIGHT):
        mode = 2 # 우회     
    elif(mission_num == MISSION.UTURN):
        mode = 3 # 유턴
    elif(mission_num == MISSION.DYN_OBS):
        mode = 4 # 동적장애물       
    elif(mission_num == MISSION.STA_OBS):
        mode = 5 # 정적 장애물        
    elif(mission_num == MISSION.PARK):
        mode = 6 # 주차
    # print("MODE:", mode)
    return mode
        

def main_loop():
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        try:
            listener()
            traffic = Traffic_callback(Traffic_Sign)
            mission = Mission_callback(Traffic_Sign, Traffic_Light)
            mode_num = Mode_callback(mission_num)
            print('status', traffic, 'Mission_num', mission, 'mode', mode_num)
        except Exception as e:
            rospy.logerr(e)
        rate.sleep()

if __name__ == '__main__':
    listener()
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Shutting down...")
 