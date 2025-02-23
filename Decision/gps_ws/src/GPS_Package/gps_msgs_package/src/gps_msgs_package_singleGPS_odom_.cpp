/* 
메인 수정자: 윤수빈 
최신 업데이트: 2025-02-16

GPS 1개 데이터 처리 및 IMU 데이터 활용 차량의 위치 및 헤딩(heading) 각도 계산 
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3.h" 
#include "geometry_msgs/Pose2D.h" 
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/transform_broadcaster.h"
#include <math.h>

#define eastReferenceAngle 90 // 현재 정동쪽을 바라볼 때 측정된 각도

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

// 전역 변수 선언
geometry_msgs::Pose2D Pose;
geometry_msgs::Pose2D utm;
std_msgs::Float32 gps_heading_angle;
std_msgs::Float32 east_angle;
nav_msgs::Odometry gps_odom1;

// ROS 퍼블리셔 전역 변수 선언
ros::Publisher angle_pub;
ros::Publisher east_angle_pub;
ros::Publisher utm_pub;
ros::Publisher gps_odom_pub;

double diff_x=0.0; double diff_y=0.0;
double lat1 = 0.0; double lon1 = 0.0; double alt1 = 0.0;

int fix = 1;

double imu_yaw = 0.0; double odom_yaw = 0.0;
double imu_yaw_offset = 0;

// 각도를 0도에서 360도 사이로 정규화
double normalizeAngle(double angle){
    while (angle < 0) { angle += 360; }
    while (angle >= 360) { angle -= 360; }
    return angle;
}

// 정동쪽을 0도로 설정하기 위한 보정
double calibrateHeadingAngle(double measuredAngle){

    /*캘리브레이션 시 마다 시작 각도 손으로 넣어주기 !*/
    double calibratedAngle = measuredAngle + eastReferenceAngle; // 보정 적용
    
    return normalizeAngle(calibratedAngle); // 정규화된 각도 반환
}

// WGS84 (세계 표준 좌표계) GPS 데이터를 UTM (Universal Transverse Mercator) 좌표계로 변환
void wgs2utm(double lat, double lon, int zone , double& east, double& north){
    double lat_rad = lat * M_PI/180;
    double lon_rad = lon * M_PI/180;

    double phi = lat_rad;
    double lambda = lon_rad;
    double lambda0 = (zone * 6 -183) * M_PI/180;
    double sm_a = 6378137;
    double sm_b = 6356752.31;

    double ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
    double nu2 = ep2*pow(cos(phi), 2.0);
    double N = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));
    double l = lambda - lambda0;
    double t = tan(phi);
    double t2 = t * t;

    double l3coef = 1 - t2 + nu2;
    double l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
    double l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
    double l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
    double l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
    double l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    east = N * cos(phi) * l + 
        (N / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) + 
        (N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) + 
        (N / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

    double n = (sm_a - sm_b) / (sm_a + sm_b);
    double alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n,4.0) / 64.0));
    double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);
    double gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n,4.0) / 32.0);
    double delta = (-35.0 * pow(n,3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
    double epsilon = (315.0 * pow(n, 4.0) / 512.0);

    double ArcLengthMeridian = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) + (delta * sin(6.0  * phi)) + (epsilon * sin(8.0 * phi)));

    north = ArcLengthMeridian + 
            (t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0)) + 
            (t / 24.0 * N * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0)) + 
            (t / 720.0 * N * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0)) + 
            (t / 40320.0 * N * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));
}

// Odometry에서 Pose와 Heading Angle을 받아오는 콜백 함수
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 위치 정보 업데이트
    Pose.x = msg->pose.pose.position.x;
    Pose.y = msg->pose.pose.position.y;

    // Heading 업데이트
    double heading_angle = normalizeAngle(RAD2DEG(yaw));

    // 동쪽 기준 보정
    double east_angle_value = calibrateHeadingAngle(heading_angle);

    // 퍼블리셔가 초기화되지 않은 경우를 방지하기 위해 확인 후 발행
    if (angle_pub) angle_pub.publish(gps_heading_angle);
    if (east_angle_pub) east_angle_pub.publish(east_angle);

    // ROS 메시지 업데이트 및 발행
    gps_heading_angle.data = heading_angle;
    east_angle.data = east_angle_value;

    ROS_INFO("North CW UTM Heading: %f degrees", heading_angle);
    ROS_INFO("EAST CW UTM Heading: %f degrees \n", east_angle_value);

    // UTM 좌표 메시지 업데이트 및 발행
    utm.x = Pose.x;
    utm.y = Pose.y;
    utm.theta = DEG2RAD(east_angle_value);
    utm_pub.publish(utm);

    // Odometry 메시지 업데이트 및 발행
    gps_odom1.header.stamp = ros::Time::now();
    gps_odom1.header.frame_id = "odom";
    gps_odom1.child_frame_id = "gps_footprint";

    gps_odom1.pose.pose.position.x = Pose.x;
    gps_odom1.pose.pose.position.y = Pose.y;
    gps_odom1.pose.pose.position.z = 0;

    gps_odom1.pose.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(heading_angle));

    gps_odom_pub.publish(gps_odom1);
}

// ----------------------------------------------------------------
int main(int argc, char **argv){
    char buf[2];
    double h_angle = 0.0;
    double utm_x = 0.0; double utm_y = 0.0;
    
    /*frame id*/
    std::string odom_frame_id = "odom";
    std::string odom_child_frame_id = "gps_footprint";
    
    ros::init(argc, argv, "single_gps_package");
    ros::NodeHandle n;
        
    ros::param::get("~imu_yaw_offset", imu_yaw_offset);
    ros::param::get("~odom_frame_id", odom_frame_id);
    ros::param::get("~odom_child_frame_id", odom_child_frame_id);  

    ros::Subscriber sub_odometry = n.subscribe("/odometry/filtered_map", 10, &odometryCallback);
    
    angle_pub = n.advertise<std_msgs::Float32>("/gps_heading_angle", 1);
    utm_pub = n.advertise<geometry_msgs::Pose2D>("/gps/utm_pos1", 1);
    east_angle_pub = n.advertise<std_msgs::Float32>("/east_heading_angle", 1);
    gps_odom_pub = n.advertise<nav_msgs::Odometry>("/nav_odom", 1);
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    
    nav_msgs::Odometry gps_odom1;
    geometry_msgs::Quaternion odom_quat;	
        
    float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
                                0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
                                0,  0,    99999, 0,     0,     0,  // covariance on gps_z
                                0,  0,    0,     99999, 0,     0,  // large covariance on rot x
                                0,  0,    0,     0,     99999, 0,  // large covariance on rot y
                                0,  0,    0,     0,     0,     0.01};  // large covariance on rot z 

    for(int i = 0; i < 36; i++){
        gps_odom1.pose.covariance[i] = covariance[i];;
    }     

    ros::Rate loop_rate(10);  // 10
    
    while (ros::ok()){	
        
        diff_x = Pose.x; diff_y = Pose.y;
        
        if(fix != -1) { // GPS 고정될 경우
            ros::spinOnce(); // 콜백 실행을 보장하기 위해 먼저 호출
            current_time = ros::Time::now();	

            //h_angle = imu_yaw + DEG2RAD(imu_yaw_offset); //IMU로 yaw 받는 버전
            h_angle = odom_yaw; // Odometry로 yaw 받는 버전
                
            Pose.theta = h_angle;
            gps_heading_angle.data = h_angle;
            angle_pub.publish(gps_heading_angle); /*IMU에서 계산된*/

            double calibratedAngle = calibrateHeadingAngle(RAD2DEG(h_angle));
            east_angle.data = calibratedAngle;
            east_angle_pub.publish(east_angle);

            //Pos -> Ros Coordinate
            ROS_INFO("GPS E: %.7lf N: %.7lf", Pose.x, Pose.y);
            ROS_INFO("NORTH Heading Angle : %.7lf",RAD2DEG(h_angle));      
            //ROS_INFO("EAST Heading Angle : %.7lf \n",calibratedAngle); 
            utm.x = Pose.x; 	utm.y = Pose.y;	utm.theta = DEG2RAD(calibratedAngle);

            utm_pub.publish(utm);
            
            gps_odom1.header.stamp = current_time;
            gps_odom1.header.frame_id = odom_frame_id;
            gps_odom1.child_frame_id = odom_child_frame_id;
        
            //odom_oriention trans to odom_quat_temp
            odom_quat = tf::createQuaternionMsgFromYaw(h_angle);//yaw trans quat
            
            gps_odom1.pose.pose.position.x, gps_odom1.pose.pose.position.y = lon1, lat1;
            gps_odom1.pose.pose.position.z = 0 ;
            gps_odom1.pose.pose.orientation = odom_quat;

            gps_odom_pub.publish(gps_odom1);
            

        
        }  
        
        loop_rate.sleep();
        ros::spinOnce();
    
    }
    return 0;
}