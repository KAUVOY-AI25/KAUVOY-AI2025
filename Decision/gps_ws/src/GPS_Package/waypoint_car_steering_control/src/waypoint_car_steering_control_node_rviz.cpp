#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h" 
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/Imu.h"
#include <chrono>  // 시간 관련 라이브러리 추가

#define DEBUG 1

#define MAX_L_STEER -24
#define MAX_R_STEER 24
#define STEER_NEUTRAL_ANGLE 0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

// waypoint를 제어하는 최소 거리 (ackermann steering 에 한해서)
#define MINIMUM_DISTANCE_WAYPOINT_CONTROL 0.5 

// 각도가 맞을 경우 직진만 함, Angle이 lock 되었다고 알려줌
#define ANGLE_LOCK  3

// 거리가 가까우면 HEADING ANGLE 제어를 줄임
#define DISTANCE_ANGLE_ANGLE_LOCK 0.5
#define LOCK_CONTROL_ANGLE  3

 
double pos_x = 0.0, pos_y = 0.0;
double roll, pitch, yaw;
double init_yaw;
int init_flag=0;
float imu_offset = 35;
double gps_heading_angle = 0;

struct Point { float x; float y; float z; };
struct WayPoints{ double x; double y; } ;

double pi_gain,pd_gain,p_gain;	
double pi_gain1,pd_gain1,p_gain1;	

double error = 0, error_old =0, error_d =0;	
double error1, error_old1, error_d1;

geometry_msgs::Pose2D my_pose,my_target_pose_goal,my_target_pose_start;

// 타이머용 전역 변수 추가
bool steering_lock = false;
auto lock_start_time = std::chrono::steady_clock::now();

// 새롭게 추가된 publisher
ros::Publisher current_pose_pub;
ros::Publisher waypoint_distance_pub;
ros::Publisher waypoint_angle_pub;
ros::Publisher target_pose_pub;

void handleSteeringLock(int &s_angle_data){
    if (steering_lock) {
        s_angle_data = 0; // 스티어링 잠금 상태인 경우, 0으로 유지
        
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_time = now - lock_start_time;
        if (elapsed_time.count() >= 0.3) {
            // 3초가 경과하면 잠금을 해제
            steering_lock = false;
        }
 
    }
}

void gps_utm_poseCallback(const geometry_msgs::Pose2D& msg){
	my_pose.x     =   msg.x;      //UTM 좌표의 경우 X,Y 좌표가 90도 회전되어 있음
	my_pose.y     =   msg.y;      //UTM 좌표의 경우 X,Y 좌표가 90도 회전되어 있음
	my_pose.theta =  msg.theta;
    //if(msg.theta <=0) my_pose.theta =  msg.theta + 2*M_PI;   
}

double low_pass_filter(double input, double input_old , double alpha){
	double result;
	result = input * (1.0-alpha) + alpha*input_old ;	

	return result ;
}

void GPSHeadingAngleCallback(const std_msgs::Float32& msg){ gps_heading_angle = msg.data; }

void gpsposeCallback(const geometry_msgs::PoseStamped& msg){
	my_pose.y =  (double)msg.pose.position.y; //-(double)msg.pose.position.x;
	my_pose.x =  (double)msg.pose.position.x; //(double)msg.pose.position.y;
    my_pose.theta = yaw;		
}

void targetPoseCallback(const geometry_msgs::Pose2D& msg){
	my_target_pose_goal.y = msg.y;
	my_target_pose_goal.x = msg.x;
	my_target_pose_goal.theta = msg.theta;	
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    tf2::Matrix3x3 m(q);      

    m.getRPY(roll, pitch, yaw);
    
    yaw = yaw + DEG2RAD(imu_offset);
    //my_pose.theta = yaw;
    //printf("%6.3lf(rad)  %6.3lf \n",yaw, yaw*180/3.14159);   
}

int heading_angle_steering_control(double c_heading_angle){
	int steer_angle; 
    /*
	error = c_heading_angle;
	error_d = error - error_old;
	steer_angle = (int)( p_gain * error + pd_gain * error_d  5+ 0.5   );		
	error_old = error;
	*/
	steer_angle = -c_heading_angle;	

	return steer_angle;	
} 


// WAYPOINT ANGLE PUBLISHER
void publishWaypointAngle(double angle){
    std_msgs::Float32 msg;
    msg.data = RAD2DEG(angle);
    waypoint_angle_pub.publish(msg);
}

// WAYPOINT DISTANCE PUBLISHER
void publishWaypointDistance(double distance){
    std_msgs::Float32 msg;
    msg.data = distance;
    waypoint_distance_pub.publish(msg);
}

// 현재 차량 위치 PUBLISHER
void publishCurrentPose() {
    geometry_msgs::Pose2D msg;
    msg.x = my_pose.x;
    msg.y = my_pose.y;
    msg.theta = my_pose.theta;
    current_pose_pub.publish(msg);
}

int main(int argc, char **argv){
        
    char buf[2];
    
    p_gain = 0.5;    pd_gain = 6; 	
    p_gain1 = 3.0;  pd_gain1 = 10.0; 
        
    ros::init(argc, argv, "utm_waypoint_car_control");

    ros::NodeHandle n;
    
    ros::param::get("/heading_control_p_gain",  p_gain);   //P  gain
    ros::param::get("/heading_control_pd_gain",  pd_gain); //pd gain
        
    ros::Subscriber sub1 = n.subscribe("/gps/utm_pos1",10, &gps_utm_poseCallback);
    ros::Subscriber sub2 = n.subscribe("/utm_my_pose",10, &gpsposeCallback);
    
    ros::Subscriber sub3 = n.subscribe("/pose_goal",10, &targetPoseCallback);
    
    ros::Subscriber sub4 = n.subscribe("/filtered_imu",10,&imuCallback);
    ros::Subscriber sub5 = n.subscribe("/gps_heading_angle",1,&GPSHeadingAngleCallback);
        
    ros::Publisher car_control_pub1 = n.advertise<std_msgs::Int32>("/steering_angle", 10);

    
    // 새롭게 추가된 publisher
    current_pose_pub = n.advertise<geometry_msgs::Pose2D>("current_pose", 10);
    waypoint_distance_pub = n.advertise<std_msgs::Float32>("waypoint_distance", 10);
    waypoint_angle_pub = n.advertise<std_msgs::Float32>("waypoint_angle", 10);
    target_pose_pub = n.advertise<geometry_msgs::Pose2D>("target_pose", 10);  // 추가됨

    printf("%6.3lf %6.3lf\n",p_gain , pd_gain);
    printf("%6.3lf %6.3lf\n",p_gain1 , pd_gain1);
    
    ros::Rate loop_rate(5);  // 10
    
    long count = 0;
    int waypoint_arrival_flag = 0 ;
    double waypoint_distance = 0.0;
    double waypoint_angle  = 0.0, waypoint_angle_old  = 0.0;
    double filtered_waypoint_angle = 0.0;
    
    std_msgs::Int32 s_angle;
    
    while (ros::ok()){	
        /*steering 각도 계산 시작*/
        double waypoint_pos_base_link_x     = 0.0;
        double waypoint_pos_base_link_y     = 0.0; 
        double waypoint_pos_base_link_theta = 0.0; 
        double tf_base_map_x = 0.0, tf_base_map_y = 0.0; 

        tf_base_map_x = my_target_pose_goal.x - my_pose.x; /*0215 이솔 수정*/
        tf_base_map_y = my_target_pose_goal.y - my_pose.y; /*0215 이솔 수정*/
        
        waypoint_pos_base_link_x = tf_base_map_x * cos(my_pose.theta)  + tf_base_map_y * sin(my_pose.theta);   // rotation_matrix
        waypoint_pos_base_link_y = -tf_base_map_x * sin(my_pose.theta) + tf_base_map_y * cos(my_pose.theta);   	

        waypoint_angle = atan2(waypoint_pos_base_link_y ,waypoint_pos_base_link_x);	// 상대좌표 계산 - 각도
        /*steering 각도 계산 마무리*/
        
        filtered_waypoint_angle = low_pass_filter(waypoint_angle, waypoint_angle_old , 0.7); // 상대좌표 계산 - 거리
        
        waypoint_angle_old =  waypoint_angle;   
        waypoint_distance = sqrt(waypoint_pos_base_link_x*waypoint_pos_base_link_x  + waypoint_pos_base_link_y*waypoint_pos_base_link_y);
        
        // PUBLISH DATA
        publishWaypointAngle(filtered_waypoint_angle);
        publishWaypointDistance(waypoint_distance);
        publishCurrentPose();
        target_pose_pub.publish(my_target_pose_goal);
        
        printf("------------- base link coordinatioin ----------------\n");
        printf("1: my pose : %lf %lf %lf(deg)\n", my_pose.x,my_pose.y,RAD2DEG(my_pose.theta));	
        printf("2: goal pose : %lf %lf \n", my_target_pose_goal.x,my_target_pose_goal.y);		
        printf("3: Way point(base_link) : %lf %lf\n", waypoint_pos_base_link_x,waypoint_pos_base_link_y);
        printf("4: WayPoint Distance %6.3lf , WayPoint Angle %6.3lf \n",waypoint_distance, RAD2DEG(waypoint_angle));	
        
        printf("=========== Heading Angle Correction Motion Enable =========== \n");
        s_angle.data = heading_angle_steering_control(RAD2DEG(filtered_waypoint_angle));
        printf("5: control Steering angle %2d\n", s_angle.data);
        car_control_pub1.publish(s_angle);
        printf("6: waypoint angle %f\n", RAD2DEG(waypoint_angle));
        printf("\n\n");

        if((RAD2DEG(waypoint_angle) >= -ANGLE_LOCK) && (RAD2DEG(waypoint_angle) <= ANGLE_LOCK)){
            if (!steering_lock) {
                    // 처음으로 잠금이 걸리는 순간, 타이머 시작
                    steering_lock = true;
                    lock_start_time = std::chrono::steady_clock::now();
                    printf("-------------------------\n");
                    printf("Heading Angle is locked!!\n");
                    printf("-------------------------\n");
            }
        }	

        int angle = s_angle.data;
        handleSteeringLock(angle);
        
        s_angle.data = angle;  // Update s_angle.data after handling  
        s_angle.data = s_angle.data % 360; // 각도 확인 할것 ROS 좌표축에 맞도록 수정했음 2022.07.10	

        ROS_INFO("Steering Angle %d", s_angle.data);
        printf("\n\n");
        
        loop_rate.sleep();
        ros::spinOnce();
        ++count;
    }
    return 0;
}

