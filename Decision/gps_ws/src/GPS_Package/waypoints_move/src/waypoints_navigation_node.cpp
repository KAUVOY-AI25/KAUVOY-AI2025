/* Waypoint를 따라 Steering Angle(조향각)과 Speed(속도)를 결정 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h" /*수빈추가 0215*/
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"   
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include <math.h>

#define WAYPOINT "/home/kauvoy/gps_ws/0219_22:37_kau_UTM.txt" // waypoint.txt 경로 설정

#define MAX_L_STEER -25
#define MAX_R_STEER 24 
#define STEER_NEUTRAL_ANGLE 0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoints_NO 407   // waypoint의 개수
#define WayPoint_X_Tor 1.5 // waypoint의 허용 오차 (m)
#define WayPoint_Y_Tor 1.5 // waypoint의 허용 오차 (m)

#define V_Region_NO  2
#define V_Speed_Region_NO 3
#define W_Region_NO  1
#define Pass_Region_NO 1
#define Park_Region_NO 1

double pos_x = 0.0, pos_y = 0.0;

int vision_steering_angle = 0, waypoint_steering_angle = 0;
int car_speed = 0;
int no_waypoints = WayPoints_NO;
bool topic_gps_datum_rcv = false;
double gps_heading_angle = 0.0, waypoint_line_angle = 0.0;
double datum_lat, datum_lon, datum_yaw; // UTM 기준점 (GPS 기준 좌표)
double datum_utm_east, datum_utm_north;
double target_utm_x = 0.0, target_utm_y = 0.0; 

// Flag 초기화
int init_flag = 0, wp_go_id = 0, wp_finish_id = 0;
double roll,pitch,yaw;

struct Point { double x, y, z; };
struct WayPoints { double x, y; };
struct Rect_Region { double top, bottom, left, right; };

geometry_msgs::Pose2D my_pose;					// 현재 차량의 위치와 방향 (x, y, theta)
geometry_msgs::Pose2D my_pose_utm_waypoint;     // 'UTM 좌표계에서' 현재 차량의 위치 또는 웨이포인트 (x, y, theta)
geometry_msgs::Pose2D my_target_pose_goal; 		// 현재 목표 지점 (x, y, theta)
geometry_msgs::Pose2D my_target_pose_goal_prev; // 이전 목표 지점 (x, y, theta)
geometry_msgs::Pose2D initial_utm_pose; 		// 차량의 초기 위치 (x, y, theta)

struct Rect_Region WayPoint_Region[W_Region_NO];
struct WayPoints my_waypoints_list[WayPoints_NO];


// ---------[Callback 함수]---------------------------------------------------------------------

//  GPS 데이터를 수신하여 차량의 현재 UTM 좌표를 my_pose에 저장
void gps_utm_poseCallback(const geometry_msgs::Pose2D& msg){
	
	my_pose.x = msg.x;
	my_pose.y = msg.y; // UTM 좌표의 경우 X,Y 좌표가 90도 회전되어 있음
	my_pose.theta = msg.theta; 
	
	//if(msg.theta <=0) my_pose.theta =  msg.theta + 2*M_PI;   
}

// GPS Datum 
void gps_datum_Callback(const geometry_msgs::Vector3::ConstPtr& msg){
	
	printf("GPS Datum 수신 완료!\n");
	topic_gps_datum_rcv = true;
	datum_lat = msg->x; datum_lon = msg->y; datum_yaw = msg->z;  
	
}

// waypoint update
void waypointstartIDCallback(const std_msgs::Int16& msg) { wp_go_id = msg.data; }
void waypointfinishIDCallback(const std_msgs::Int16& msg) { wp_finish_id = msg.data; }

// GPS Heading Angle (북쪽 기준 헤딩각 <= gps_msgs_package_singleGPS.cpp)
void GPSHeadingAngleCallback(const std_msgs::Float32& msg){ gps_heading_angle = msg.data; /* 라디안 */}

// Odometry callback
void odomCallback(const nav_msgs::Odometry& msg){

	my_pose.x = (double)msg.pose.pose.position.x;
	my_pose.y = (double)msg.pose.pose.position.y;
	
	tf2::Quaternion q( msg.pose.pose.orientation.x,        msg.pose.pose.orientation.y,
					   msg.pose.pose.orientation.z,        msg.pose.pose.orientation.w );
	tf2::Matrix3x3 m(q);     

	m.getRPY(roll, pitch, yaw);
	my_pose.theta = yaw;	

}

// ---------- [ waypoint 로드 & 변환 ] -------------------------------------------------------

int current_waypoint_id = 0; // 새로운 waypoint 갱신을 위한 콜백 함수

void init_waypoint_region(void){
	FILE *fp;
	
	fp= fopen(WAYPOINT,"r");
	
	if(fp == NULL){
		ROS_INFO("Waypoint_region does not exit!");
		WayPoint_Region[0].top  =  7.6; WayPoint_Region[0].bottom =  7.2;
		WayPoint_Region[0].left = -4.9; WayPoint_Region[0].right = -5.1;
	} else {
		fscanf(fp,"%lf %lf %lf %lf",&WayPoint_Region[0].top, &WayPoint_Region[0].bottom, &WayPoint_Region[0].left, &WayPoint_Region[0].right);
		fclose(fp);
	}
}

// waypoint를 txt파일에서 불러와서 초기화
void init_waypoint(void){

	FILE *fp;
	
	fp= fopen(WAYPOINT,"r");
	
	if(fp == NULL){ // 파일이 없으면 기본 waypoint 설정 (사실상 에러문)
		ROS_INFO("Waypoints_data does not exit!");
		my_waypoints_list[0] = {1, 1};   
		my_waypoints_list[1] = {3, 3};
		my_waypoints_list[2] = {2, 6};  		
		my_waypoints_list[3] = {3, 10};
		
		no_waypoints = 4;
		wp_finish_id = no_waypoints;
	}else{ // 로드할 waypoint.txt
		fscanf(fp,"%d",&no_waypoints); 
		
		wp_finish_id = no_waypoints;
	
		for(int i=0; i < no_waypoints; i++){
			fscanf(fp,"%lf %lf",&my_waypoints_list[i].x, &my_waypoints_list[i].y);
		}
	
		ROS_INFO("WayPoints Number %d", WayPoints_NO);

		for(int i=0; i < no_waypoints; i++){
			ROS_INFO("WayPoints-%d : [%.2lf | %.2lf]", i, my_waypoints_list[i].x, my_waypoints_list[i].y);
		}

		fclose(fp);
	}
}

// -----------------------------------------------------------------------------------

void WaySteerControlCallback(const std_msgs::Int16& angle){

	waypoint_steering_angle = (int)(angle.data) ;
	
	if(waypoint_steering_angle >= MAX_R_STEER)  waypoint_steering_angle = MAX_R_STEER;
	if(waypoint_steering_angle <= MAX_L_STEER)  waypoint_steering_angle = MAX_L_STEER;  

}

void waypoint_tf(void){

	double x,y;
	double tf_waypoint_x,tf_waypoint_y; 
	x = y = 0;
	
	tf_waypoint_x = my_pose.x - my_target_pose_goal_prev.x;
	tf_waypoint_y = my_pose.y - my_target_pose_goal_prev.y;  
		
	// 회전 행렬	
	my_pose_utm_waypoint.x = tf_waypoint_x * cos(waypoint_line_angle) + tf_waypoint_y * sin(waypoint_line_angle);   
	my_pose_utm_waypoint.y = - tf_waypoint_x * sin(waypoint_line_angle) + tf_waypoint_y * cos(waypoint_line_angle);   	
	
}

void base_link_tf_utm(void){

	double waypoint_pos_base_link_x     = 0.0; double waypoint_pos_base_link_y     = 0.0; 
	double waypoint_pos_base_link_theta = 0.0; 
	double tf_base_map_x, tf_base_map_y; 
	double waypoint_angle, waypoint_distance;	 
	
	//상대좌표로 변환  no translation
	tf_base_map_x = -my_pose.x + my_target_pose_goal.x; 
	tf_base_map_y = -my_pose.y + my_target_pose_goal.y; 

	waypoint_pos_base_link_y = (  tf_base_map_x * cos(my_pose.theta) + tf_base_map_y * sin(my_pose.theta) ); //*-1;   // rotation_matrix 90도 회전
	waypoint_pos_base_link_x = ( -tf_base_map_x * sin(my_pose.theta) + tf_base_map_y * cos(my_pose.theta) ); //* 1;   		

	waypoint_angle = atan2(waypoint_pos_base_link_y ,waypoint_pos_base_link_x);	
	waypoint_distance = sqrt(waypoint_pos_base_link_x * waypoint_pos_base_link_x  + waypoint_pos_base_link_y * waypoint_pos_base_link_y);
	
	ROS_INFO(" X : %6.3lf   Y : %6.3lf  Yaw : %6.3lf ", my_pose.x, my_pose.y, RAD2DEG(my_pose.theta)); 
	ROS_INFO(" b_x : %6.3lf  b_y : %6.3lf", waypoint_pos_base_link_x, waypoint_pos_base_link_y);  
		
}


// waypoint로 가는 경로 상에 있는지 체크 (어디쓰는거지?)
void check_inside_waypoint(int waypoint_id){

	double  waypt_line_angle = 0.0;
	
	//printf("%d %d \n", wp_go_id, no_waypoints);
	if(waypoint_id != 0){ 
		my_target_pose_goal_prev.x = my_waypoints_list[waypoint_id-1].x ;//- initial_utm_pose.x;
		my_target_pose_goal_prev.y = my_waypoints_list[waypoint_id-1].y ;//- initial_utm_pose.x;
		
		double delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
		double delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
		waypt_line_angle = atan2(delta_y, delta_x);
		
		printf("1: tf angle %lf\n", RAD2DEG( waypt_line_angle)); 
	}

}

/*
// ------------[ Visualization ]------------------------

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

visualization_msgs::Marker Draw_Marker_point(float x, float y){
	
	visualization_msgs::Marker marker_point;
	float x_p = 0.0, y_p = 0.0;
	double zone;
	if(datum_lon < 0)
	{
        zone = (datum_lon + 180) / 6 + 1;
    }
    else
    {
         zone = datum_lon / 6 + 31;
    }
    wgs2utm( datum_lat,datum_lon, zone, datum_utm_east, datum_utm_north);
	   
	datum_utm_east = datum_utm_east * 0.9996 + 500000;
	datum_utm_north = datum_utm_north * 0.9996;
	
	x_p = x - datum_utm_east;
	y_p = y - datum_utm_north;
	
	marker_point.header.frame_id = "utm"; // utm frame 기준
    marker_point.header.stamp = ros::Time::now();
    marker_point.type = visualization_msgs::Marker::SPHERE;
    marker_point.id = 400;
    marker_point.action = visualization_msgs::Marker::ADD;
    marker_point.pose.orientation.w = 1.0;
	
	marker_point.pose.position.x = x_p;
	marker_point.pose.position.y = y_p;
	marker_point.color.r = 1.0;
	marker_point.color.g = 0.0;
	marker_point.color.b = 0.0;
    marker_point.color.a = 1.0;
	return marker_point;
}

nav_msgs::Path draw_target_line(ros::Time current_time){
	nav_msgs::Path target_line_path1;
	double zone;
	if(datum_lon < 0)
	{
        zone = (datum_lon + 180) / 6 + 1;
    }
    else
    {
         zone = datum_lon / 6 + 31;
    }
    wgs2utm( datum_lat,datum_lon, zone, datum_utm_east, datum_utm_north);
	   
	datum_utm_east = datum_utm_east * 0.9996 + 500000;
	datum_utm_north = datum_utm_north * 0.9996;
	   
	   
	target_line_path1.header.stamp=current_time;
    target_line_path1.header.frame_id="utm";
    
    target_line_path1.poses.clear();
  
	geometry_msgs::PoseStamped this_pose_stamped;
     
    this_pose_stamped.pose.position.x = my_pose.x - datum_utm_east;
    this_pose_stamped.pose.position.y = my_pose.y - datum_utm_north;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(waypoint_line_angle);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

 
    target_line_path1.poses.push_back(this_pose_stamped);     
    this_pose_stamped.pose.position.x =  target_utm_x - datum_utm_east ;
    this_pose_stamped.pose.position.y =  target_utm_y - datum_utm_north;
 

    goal_quat = tf::createQuaternionMsgFromYaw(waypoint_line_angle);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="utm";
    
    
    //printf("datum: %6.3lf %6.3lf", datum_lat, datum_lon);
    target_line_path1.poses.push_back(this_pose_stamped);
     
    return target_line_path1;
}
*/

// -------------------------------------------------------------------------

int main(int argc, char **argv){
	
	char buf[2];
	ros::init(argc, argv, "cleanbot_race_waypoints_manager_utm");

	ros::NodeHandle n;
	
	std_msgs::Int16 s_angle;
	std_msgs::Int16 c_speed;
	std_msgs::Int16 ros_waypoint_id; // 현재 Waypoint ID
	std_msgs::String slam_reset;
	geometry_msgs::Pose2D gps_init_pose2d_data;
	nav_msgs::Path target_line_path; 

	slam_reset.data = "reset";
	datum_lat = datum_lon =  datum_yaw = 0.0; 
	
	ros::Subscriber sub1 = n.subscribe("/Car_Control_cmd/W_SteerAngle_Int16",10, &WaySteerControlCallback);
	ros::Subscriber sub2 = n.subscribe("/gps/utm_pos1",10, &gps_utm_poseCallback);

	ros::Subscriber sub_gps_datum = n.subscribe("/gps/datum",1,&gps_datum_Callback);  // 

	ros::Subscriber sub3 = n.subscribe("/start_waypoint_id_no",1, &waypointstartIDCallback);   // 어디서?
	ros::Subscriber sub4 = n.subscribe("/finish_waypoint_id_no",1, &waypointfinishIDCallback); // 어디서?
	ros::Subscriber sub5 = n.subscribe("/gps_heading_angle",1,&GPSHeadingAngleCallback); // (북쪽 기준 헤딩각)
	
	ros::Publisher car_control_pub1 = n.advertise<std_msgs::Int16>("/Car_Control_cmd/A_SteerAngle_Int16", 10);
	ros::Publisher car_control_pub2 = n.advertise<std_msgs::Int16>("/Car_Control_cmd/A_Speed_Int16", 10);
	//ros::Publisher car_control_pub3 = n.advertise<std_msgs::Int32>("/steering_angle", 10);  // 수빈추가: Int32 타입 퍼블리셔 -> ERP 메세지 형식에 맞춤
	
	ros::Publisher target_id_pub    = n.advertise<std_msgs::Int16>("/target_id",2);
	ros::Publisher target_pos_pub   = n.advertise<geometry_msgs::Pose2D>("/pose_goal", 10);
	
	ros::Publisher target_guide_line_pub = n.advertise<nav_msgs::Path>("/target_guide_line",1, true);
	ros::Publisher waypoint_guide_line_pub = n.advertise<nav_msgs::Path>("/waypoint_guide_line",1, true);

	ros::Rate loop_rate(5);  // 10 
	
	long count = 0;
	int mission_flag[WayPoints_NO] = {0,};
	double pos_error_x = 0.0, pos_error_y = 0.0;

	double waypoint_distance = 0.0;
	double waypoint_gap_distance = 0.0;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	geometry_msgs::Pose2D pose_goal;  
		
	init_waypoint(); 
	
	initial_utm_pose.x, initial_utm_pose.y = datum_lat, datum_lon;  //gps datum 처리 

	int vision_id = -1;
	int vision_speed_id = -1;
	int waypoint_id = 0;
	
	double start_x = 0., start_y = 0.;  

	init_waypoint_region();
	
	double delta_x, delta_y ;
	delta_x = delta_y = 0.0;
	
	double base_line_a, base_line_b; // waypoint line between start and target point
	
	pose_goal.x = my_waypoints_list[wp_go_id].x;
	pose_goal.y = my_waypoints_list[wp_go_id].y;
	pose_goal.theta = DEG2RAD(0);
	
	target_pos_pub.publish(pose_goal);   
	
	
	if(count == 0){
		start_x = my_pose.x;   start_y = my_pose.y;
		start_x = 0;   start_y = 0;
	}  
				
	ROS_INFO(" ");
	ROS_INFO("\n\n\n\nutm relative mode\n\n\n\n");  
	ROS_INFO(" ");
	printf("topic_gps_datum_mode  : %d\n", topic_gps_datum_rcv );

	ros::Duration(3.0).sleep() ;

	while (ros::ok()){	
		
		gps_init_pose2d_data.x = my_waypoints_list[0].x;
		gps_init_pose2d_data.y = my_waypoints_list[0].y;
		gps_init_pose2d_data.theta = 0;
				
		if(waypoint_id!= -1){	
			
			my_target_pose_goal.x = my_waypoints_list[wp_go_id].x ; //- initial_utm_pose.x;
			my_target_pose_goal.y = my_waypoints_list[wp_go_id].y ; //- initial_utm_pose.y;
			
			if(wp_go_id == 0){
				delta_x = my_waypoints_list[1].x - my_waypoints_list[0].x;
				delta_y = my_waypoints_list[1].y - my_waypoints_list[0].y;
				
				my_target_pose_goal_prev.x = my_waypoints_list[0].x - delta_x;
				my_target_pose_goal_prev.y = my_waypoints_list[0].y - delta_y;
				
				delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
				delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
				waypoint_line_angle = atan2(delta_y, delta_x);
			}else{ 		  
				my_target_pose_goal_prev.x = my_waypoints_list[wp_go_id-1].x; // - initial_utm_pose.x;
				my_target_pose_goal_prev.y = my_waypoints_list[wp_go_id-1].y; // - initial_utm_pose.y;
				
				delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
				delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
				waypoint_line_angle = atan2(delta_y, delta_x);
				
			// printf("1: angle %lf\n", RAD2DEG(waypoint_line_angle)); 	        
			}
					
			waypoint_distance = sqrt(delta_x*delta_x + delta_y*delta_y);		
			waypoint_gap_distance = sqrt(delta_x*delta_x + delta_y*delta_y) - WayPoint_X_Tor;
			//printf("gap : %6.3lf  \n",waypoint_gap_distance);
			printf("1: angle %lf\n", RAD2DEG(waypoint_line_angle)); 	
			
			pos_error_x = abs(my_pose.x - my_waypoints_list[wp_go_id].x);
			pos_error_y = abs(my_pose.y - my_waypoints_list[wp_go_id].y); 
				
			pose_goal.x =  my_waypoints_list[wp_go_id].x ;;//- initial_utm_pose.y;    // umt coordinate
			pose_goal.y =  my_waypoints_list[wp_go_id].y;; // + initial_utm_pose.x);   // umt coordinate 
			
			pose_goal.theta = DEG2RAD(0);
			
			waypoint_tf();
			base_link_tf_utm();
			
			target_pos_pub.publish(pose_goal);
			ros_waypoint_id.data  = wp_go_id;
			
			ROS_INFO("[%3d] WayPoint goal X : %6.3lf,  goal Y : %6.3lf ",wp_go_id, my_target_pose_goal.x, my_target_pose_goal.y);  
			ROS_INFO("pos_error_x : %f , pos_error_y : %f", pos_error_x, pos_error_y);
			if((count >= 0) && (pos_error_x <= WayPoint_X_Tor) && (pos_error_y <= WayPoint_Y_Tor )){           
				c_speed.data=0;
				
				printf("----------------------------\n"); 
				printf(" Arrvied at My WayPoint[%3d] !\n", wp_go_id); 
				printf("----------------------------\n"); 
				
				//ros::Duration(4.0).sleep() ;      
				count = -3;
				wp_go_id++;
			}
			
			s_angle.data = -waypoint_steering_angle;
			
			c_speed.data = 45;
			
			if(wp_go_id >= wp_finish_id){
				c_speed.data = 0;
				wp_go_id = wp_finish_id;

				ROS_INFO("WayPoint Mission Completed !");	
			}
		}	

		target_id_pub.publish(ros_waypoint_id);
		ROS_INFO("Steering_angle : %d, Speed : %d \n", s_angle.data , c_speed.data);

		std_msgs::Int32 s_angle_int32;  // 추가: Int32 메시지 변수 생성 0215
    	s_angle_int32.data = s_angle.data;  // 기존 Int16 값을 Int32에 복사 0215
		
		if(count>=2){
			car_control_pub1.publish(s_angle);
			car_control_pub2.publish(c_speed);
			//car_control_pub3.publish(s_angle_int32);  // 추가: steering_angle을 Int32 형태로 퍼블리시 0215
		}
		else{
			c_speed.data = 35;
			car_control_pub2.publish(c_speed);
			//car_control_pub3.publish(s_angle_int32);  // 추가: 초기에도 퍼블리시
		}	

		/*
		target_line_path = draw_target_line(current_time);
	    target_guide_line_pub.publish(target_line_path);
		*/

		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}

	return 0;

}
