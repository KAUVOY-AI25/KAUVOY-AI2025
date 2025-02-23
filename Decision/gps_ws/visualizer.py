#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

# 파일 경로 설정 (waypoint TXT 파일)
WAYPOINT_FILE = "/home/kauvoy/gps_ws/0219_22:37_kau_UTM_v.txt"

# 실시간 변수 저장
current_pose = Pose2D()
target_pose = Pose2D()
waypoint_distance = 0.0
waypoint_angle = 0.0
waypoints = []
origin_x, origin_y = None, None  # 기준 좌표 (첫 번째 waypoint)

def read_waypoints():
    """ TXT 파일에서 waypoints를 읽어와 저장 """
    global waypoints, origin_x, origin_y
    waypoints.clear()  # 기존 데이터 초기화

    try:
        with open(WAYPOINT_FILE, "r") as f:
            lines = f.readlines()
            for line in lines:
                parts = line.strip().split()
                if len(parts) == 2:  # (x, y) 형태인 경우만 추가
                    x, y = float(parts[0]), float(parts[1])
                    waypoints.append((x, y))

            if waypoints:
                # 기준점 설정 (첫 번째 waypoint를 원점으로)
                origin_x, origin_y = waypoints[0]
                # 모든 waypoints를 로컬 좌표로 변환
                waypoints = [(x - origin_x, y - origin_y) for x, y in waypoints]

    except Exception as e:
        rospy.logerr(f"Waypoints 파일을 읽는 중 오류 발생: {e}")

def pose_callback(msg):
    """ 현재 차량 위치를 기준 좌표계로 변환하여 저장 """
    global current_pose
    current_pose.x = msg.x - origin_x if origin_x else msg.x
    current_pose.y = msg.y - origin_y if origin_y else msg.y
    current_pose.theta = msg.theta  # 헤딩 각도 저장

def target_pose_callback(msg):
    """ 목표 Waypoint 위치를 기준 좌표계로 변환하여 저장 """
    global target_pose
    target_pose.x = msg.x - origin_x if origin_x else msg.x
    target_pose.y = msg.y - origin_y if origin_y else msg.y
    target_pose.theta = msg.theta

def distance_callback(msg):
    """ WAYPOINT GOAL까지의 거리 저장 """
    global waypoint_distance
    waypoint_distance = msg.data

def angle_callback(msg):
    """ WAYPOINT ANGLE 저장 """
    global waypoint_angle
    waypoint_angle = np.deg2rad(msg.data)  # degree → radian 변환

def visualize():
    """ 실시간 데이터 시각화 """
    plt.ion()  # Interactive mode 활성화
    fig, ax = plt.subplots()

    while not rospy.is_shutdown():
        ax.clear()

        if waypoints:
            # 로컬 좌표 변환된 Waypoint 데이터
            x_vals, y_vals = zip(*waypoints)
            x_min, x_max = min(x_vals), max(x_vals)
            y_min, y_max = min(y_vals), max(y_vals)

            # 그래프 스케일 조정 (현재 차량 위치 & 목표 Waypoint 포함)
            x_min = min(x_min, current_pose.x, target_pose.x) - 10
            x_max = max(x_max, current_pose.x, target_pose.x) + 10
            y_min = min(y_min, current_pose.y, target_pose.y) - 10
            y_max = max(y_max, current_pose.y, target_pose.y) + 10

            #ax.set_xlim(x_min, x_max)
            #ax.set_ylim(y_min, y_max)
            ax.set_xlim(-65, 65)
            ax.set_ylim(-105, 45)
        else:
            # Waypoints가 없을 경우 기본 축 범위 설정
            ax.set_xlim(-50, 50)
            ax.set_ylim(-50, 50)

        ax.set_title("Waypoint & Vehicle Visualization")

        # 현재 차량 위치 (파란 점)
        ax.plot(current_pose.x, current_pose.y, 'bo', markersize=10, label="Car Position")

        # 목표 Waypoint 위치 (빨간 별)
        ax.plot(target_pose.x, target_pose.y, 'r*', markersize=12, label="Target Pose")

        # Waypoints & 번호 추가
        if waypoints:
            ax.plot(x_vals, y_vals, 'rx', markersize=5, label="Waypoints")
            for i, (x, y) in enumerate(waypoints):
                ax.text(x + 0.5, y + 0.5, str(i + 1), fontsize=10, color='black')  # Waypoint 번호 표시

        # 현재 차량의 헤딩 방향 화살표 (Theta 값 이용)
        arrow_length = 6  # 화살표 길이
        theta_rad = current_pose.theta  # 현재 차량의 헤딩 각도 (라디안)
        dx = arrow_length * np.cos(theta_rad)  # X축 이동 방향
        dy = arrow_length * np.sin(theta_rad)  # Y축 이동 방향
        ax.quiver(current_pose.x, current_pose.y, dx, dy, angles='xy', scale_units='xy', scale=0.5, color="b", label="Heading")  

        # Waypoint Angle 방향 화살표 (기준점: 현재 차량 위치)
        if waypoint_angle != 0:
            waypoint_dx = arrow_length * np.cos(waypoint_angle)
            waypoint_dy = arrow_length * np.sin(waypoint_angle)
            ax.quiver(current_pose.x, current_pose.y, waypoint_dx, waypoint_dy, angles='xy', scale_units='xy', scale=0.5, color="g", label="Waypoint Angle VECTOR")

        # (현재 헤딩 + Waypoint Angle) 방향 화살표 (보라색)
        corrected_angle = theta_rad + waypoint_angle
        corrected_dx = arrow_length * np.cos(corrected_angle)
        corrected_dy = arrow_length * np.sin(corrected_angle)
        ax.quiver(current_pose.x, current_pose.y, corrected_dx, corrected_dy, angles='xy', scale_units='xy', scale=0.5, color="m", label="corrected heading = heading-w_heading")  

        # 목표 WAYPOINT까지의 거리 표시
        ax.text(0.3, 0.95, f"Distance to Goal: {waypoint_distance:.2f}m", transform=ax.transAxes, fontsize=12)
        ax.text(0.3, 0.9, f"Heading Angle: {np.rad2deg(theta_rad):.2f}°", transform=ax.transAxes, fontsize=12)
        # ANGLE 표시
        ax.text(0.3, 0.85, f"Waypoint Angle: {np.rad2deg(waypoint_angle):.2f}°", transform=ax.transAxes, fontsize=12)
        ax.text(0.3, 0.8, f"Steering Angle: {np.rad2deg(-waypoint_angle):.2f}°", transform=ax.transAxes, fontsize=12)

        #ax.legend()
        plt.pause(0.1)

def main():
    rospy.init_node("waypoint_visualizer", anonymous=True)

    # Subscribers
    rospy.Subscriber("/current_pose", Pose2D, pose_callback)
    rospy.Subscriber("/target_pose", Pose2D, target_pose_callback)
    rospy.Subscriber("/waypoint_distance", Float32, distance_callback)
    rospy.Subscriber("/waypoint_angle", Float32, angle_callback)

    read_waypoints()
    visualize()

if __name__ == "__main__":
    main()

