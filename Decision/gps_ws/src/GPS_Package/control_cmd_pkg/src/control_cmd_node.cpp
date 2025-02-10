#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include <math.h>
#include <visualization_msgs/MarkerArray.h>

//#include "sensor_msgs/LaserScan.h"
//#include "yolo_msgs/Detection.h" // yolo_msgs 헤더 파일 포함

#define Lidar_Obstacle_Range 3.0
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

struct ControlCmd {
    int steer_angle;
    int speed;
    int target_id;
};

ControlCmd control_cmd;  // ControlCmd 구조체 사용

ros::Timer speed_recovery_timer;
bool speed_recovery_active = false;

void recoverSpeedCallback(const ros::TimerEvent&) {
    control_cmd.speed = 80;  // 속도 복구
    speed_recovery_active = false;  // 타이머 종료 후 플래그 초기화
    ROS_INFO("Speed recovered to 80");
}
void applyControlCommands() {
    // 차량 제어 명령을 적용하는 로직
}

void WaySteerControlCallback(const std_msgs::Int16& angle){
    control_cmd.steer_angle = angle.data;
    
}

void SpeedControlCallback(const std_msgs::Int16& speed){
    control_cmd.speed = speed.data;
}

void TargetIDCallback(const std_msgs::Int16& target_id){
    control_cmd.target_id = target_id.data;
}

///////////////////////////////mission algorithm////////////////////////////////

float original_speed = 40;  // 원래 속도를 저장할 변수
bool obstacle_detected = false;  // 장애물 감지 여부


bool is_stopped = false; // 정지 상태 플래그

ros::Time speed_set_time;    // 속도 설정 시간
bool is_speed_fixed = false;

int no13=0;
void slope() {

     if (is_speed_fixed) {
        // 속도가 고정된 상태라면 4초 동안 유지
        if (ros::Time::now() - speed_set_time < ros::Duration(4)) {
            control_cmd.speed = 37;  // 4초 동안 속도를 37로 고정
        } else {
            control_cmd.speed = 85;  // 4초 후 속도 복구
            is_speed_fixed = false;  // 속도 고정 해제
        }
        return;  // 다른 target_id는 무시
    }

    if (control_cmd.target_id == 12) { // 경사로 출발 지점
        control_cmd.speed = 85;
    } 
    else if (control_cmd.target_id == 14) { // 정상 살짝 이후 속도 유지 1: 25
        control_cmd.speed = 85;
    } 
    else if (control_cmd.target_id == 15) { // 정상 살짝 이후 속도 유지 2: 0
        control_cmd.speed = 45;
    } 
    else if (control_cmd.target_id == 16 || 
             control_cmd.target_id == 17 || 
             control_cmd.target_id == 18) { // 내리막길, 속도 -15
        control_cmd.speed = -15;
    } 
    else if (control_cmd.target_id == 19 || 
             control_cmd.target_id == 20) { // 내리막길 중간~경사 끝
        control_cmd.speed = -15;
    } 
    else if (control_cmd.target_id == 13 && !is_speed_fixed) { // 경사로 중앙 지점
        no13++;
        control_cmd.speed = 45;       // 속도 37로 설정
        speed_set_time = ros::Time::now(); // 현재 시간 기록
        if(no13==1){
        is_speed_fixed = true;    }
        else(control_cmd.speed=85);    // 속도 고정 플래그 활성화
        ROS_INFO("Speed set to 37 for 4 seconds");
    }/*
    else if (control_cmd.target_id == 13) { // 경사로 중앙 지점, 정지
        printf("1");
        control_cmd.speed = 37; // 35~40
        ros::Duration(4).sleep();  // 4초간 정지
        control_cmd.speed = 80;     // 속도 복구
    } */
    // else if (control_cmd.target_id == 21) { // 경사로 벗어남
    //     control_cmd.speed = 40;
    // }
}

std::string traffic_light_status = "unknown";  // 신호등 상태 저장
ros::Time stop_end_time;  // 정지 시간이 끝나는 시각 저장
bool is_stopping = false;  // 차량이 정지 중인지 여부

// 토픽에서 신호등 상태를 수신하는 콜백 함수
void TrafficLightCallback(const std_msgs::String::ConstPtr& msg) {
    traffic_light_status = msg->data;  // 문자열 메시지에서 신호등 상태

    // 신호등 상태에 따라 정지 시간 설정
    if (traffic_light_status == "red" || traffic_light_status == "yellow") {
        stop_end_time = ros::Time::now() + ros::Duration(2);
        is_stopping = true;  // 정지 상태로 설정
    }
}

void cross1() {  // 직진
    if (control_cmd.target_id == 83) { 
        ros::Time current_time = ros::Time::now();  // 현재 시간

        if (is_stopping && current_time < stop_end_time) {
            // 정지 시간이 아직 남아있다면 차량 정지
            control_cmd.speed = 0;
        } else {
            // 정지 시간이 끝났다면 다시 주행
            if (traffic_light_status == "red" || traffic_light_status == "yellow") {
                control_cmd.speed = 0;  
                is_stopping = true;
                stop_end_time = current_time + ros::Duration(2.0);  // 2초간 정지 유지

        
            } 
            
                else {
                  // 기존 속도로 주행
                is_stopping = false;
            }
        }
        
        // 차량 속도 및 방향 제어 명령을 적용
        applyControlCommands();
    }
    if (control_cmd.target_id >= 84 && control_cmd.target_id <= 89){
                control_cmd.speed = 50;

            }
}


void cross2() {  // 직진
    if (control_cmd.target_id == 158) { 
        ros::Time current_time = ros::Time::now();  // 현재 시간

        if (is_stopping && current_time < stop_end_time) {
            // 정지 시간이 아직 남아있다면 차량 정지
            control_cmd.speed = 0;
        } else {
            // 정지 시간이 끝났다면 다시 주행
            if (traffic_light_status == "red" || traffic_light_status == "yellow") {
                control_cmd.speed = 0;  
                is_stopping = true;
                stop_end_time = current_time + ros::Duration(2.0);  // 2초간 정지 유지
            } 
            
            else {
                // 기존 속도로 주행
                is_stopping = false;
            }
        }

        // 차량 속도 및 방향 제어 명령을 적용
        applyControlCommands();
    }
    if (control_cmd.target_id >= 159 && control_cmd.target_id <= 161){
                control_cmd.speed = 50;

            }
}

int no246=0;

void cross3() {  // 좌회전
    if (control_cmd.target_id == 246 && no246==0) { 
        ros::Time current_time = ros::Time::now();  // 현재 시간

        // 빨간불 또는 노란불 또는 초록불 일 때 차량 정지
        if (is_stopping && current_time < stop_end_time) {
            // 정지 시간이 아직 남아있다면 차량 정지
            control_cmd.speed = 0;
        } else {
            // 자회전 신호가 감지되면 원래 속도로 주행
            if(traffic_light_status == "unknown"){
                control_cmd.speed=0;
            }
            else if (traffic_light_status == "left") {
                 no246++;
                is_stopping = false;
            } else {
                // 빨간불 또는 노란불일 때 정지
                if (traffic_light_status == "red" || traffic_light_status == "yellow") {
                    control_cmd.speed = 0;  
                    is_stopping = true;
                    stop_end_time = current_time + ros::Duration(2.0);  // 2초간 정지 유지
                } else {
                     // 기존 속도로 주행
                    is_stopping = false;
                }
            }
        }

        // 차량 속도 및 방향 제어 명령을 적용
        applyControlCommands();
    }
}

void parking1(){ // T자 주차
    if (control_cmd.target_id == 177) { //T자 진입
        control_cmd.speed = -35;
    }
    else if (control_cmd.target_id == 178) { //T자 들어가기, 후진 자회전
        control_cmd.speed = -35;
        control_cmd.steer_angle = -15;  // 후진, 왼쪽 조향각
    }
    else if (control_cmd.target_id == 179) { // 후진,직진
        control_cmd.speed = -35;
        control_cmd.steer_angle = 0; 
    }
    else if (control_cmd.target_id == 180) { // 직진. 오른쪽 조향각
        control_cmd.speed = 0;
        control_cmd.steer_angle = 0;
    }
    
}

void wood() {
    if (control_cmd.target_id == 284) {
         
        control_cmd.speed = 50;    // 속도 증가
    }
    else if (control_cmd.target_id == 287) {
        control_cmd.speed = 0 ;  // 원래 속도로 복귀
    }
    applyControlCommands();
}

void parking2(){
    if (control_cmd.target_id == 366) { //평행주차 진입
        control_cmd.speed = -35;
        control_cmd.steer_angle = -6;
    }
    else if (control_cmd.target_id == 367) { //평행구간 중간지점 진입
        control_cmd.speed = -35;
        control_cmd.steer_angle = -3;  // 후진, 직진
    }
    else if (control_cmd.target_id == 368) { // 전진, 직진 (??????62, 63 뭔가이상함)
        control_cmd.speed = -35;
        control_cmd.steer_angle = 0;
    }
    else if (control_cmd.target_id == 369) { //평행구간탈출
        control_cmd.speed = 0;
        control_cmd.steer_angle = 0; //전진, 자회전
    }
    
}

void stop() {
    // 도착 지점에 도달했는지 확인하는 조건
    if (control_cmd.target_id == 414) {  // waypoint가 도착 지점과 일치하는지 확인
        control_cmd.speed = 0; 
        applyControlCommands(); 
    }
}


///////////////////////////////////main/////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_cmd_node");
    ros::NodeHandle n;
    //
    ros::init(argc, argv, "final_node");
    ros::init(argc, argv, "obstacle_detection_node");
    //ros::NodeHandle nh;

    // 차량의 원래 속도를 저장
    //original_speed = control_cmd.speed;
    original_speed = 40;


    // Subscriber
    ros::Subscriber steer_sub = n.subscribe("Car_Control_cmd/A_SteerAngle_Int16", 10, WaySteerControlCallback);
    ros::Subscriber speed_sub = n.subscribe("Car_Control_cmd/A_Speed_Int16", 10, SpeedControlCallback);
    ros::Subscriber target_id_sub = n.subscribe("target_id", 2, TargetIDCallback);
    ros::Subscriber traffic_light_sub = n.subscribe("traffic_light_status", 1000, TrafficLightCallback); //신호인식 구독
    // ros::Subscriber lidar_sub = n.subscribe("/cluster_markers", 1000, &scanCallback); // LiDAR 데이터 구독
    

    // Publisher
    ros::Publisher car_control_pub1 = n.advertise<std_msgs::Int16>("Car_Control_cmd/SteerAngle_Int16", 10);
    ros::Publisher car_control_pub2 = n.advertise<std_msgs::Int16>("Car_Control_cmd/Speed_Int16", 10);
    ros::Publisher target_id_pub = n.advertise<std_msgs::Int16>("target_id", 2);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std_msgs::Int16 steer_msg;
        std_msgs::Int16 speed_msg;
        std_msgs::Int16 target_id_msg;

        // ROS_INFO_STREAM(traffic_light_status);

        // slope();
        // cross1();
        // cross2();
        // cross3();
        // parking1();
        // wood();
        // //fast();
        // parking2();
        // stop();
        
        steer_msg.data = control_cmd.steer_angle;
        speed_msg.data = control_cmd.speed;
        target_id_msg.data = control_cmd.target_id;

        // Publish control commands
        car_control_pub1.publish(steer_msg);
        car_control_pub2.publish(speed_msg);
        target_id_pub.publish(target_id_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
