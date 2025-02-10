/* 듀얼 GPS 데이터 처리 및 IMU 데이터 활용
   차량의 위치 및 헤딩(heading) 각도 계산 */

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


#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

geometry_msgs::Pose2D Pose1; //Rear GPS
geometry_msgs::Pose2D Pose2; //Front GPS
geometry_msgs::Pose2D utm1; //Front GPS utm
geometry_msgs::Pose2D utm2; //Front GPS utm 

std_msgs::Float32 gps_heading_angle;
std_msgs::Float32 east_angle;
std::string gps1_topic, gps2_topic;

double diff_x=0.0; 
double diff_y=0.0;
double lat1 = 0.0;
double lon1 = 0.0;
double alt1 = 0.0;

double lat2 = 0.0;
double lon2 = 0.0;
double alt2 = 0.0;

double imu_yaw = 0.0;

int fix = 0;
int fix1 = 1;
int fix2 = 0;
int use_imu_yaw_angle = 0;
int use_two_gps = 0;
double imu_yaw_offset = 0;

// 각도를 0도에서 360도 사이로 정규화
double normalizeAngle(double angle) {
    while (angle < 0) {
        angle += 360;
    }
    while (angle >= 360) {
        angle -= 360;
    }
    return angle;
}

// 정동쪽을 0도로 설정하기 위한 보정
double calibrateHeadingAngle(double measuredAngle) {

    /*캘리브레이션 시 마다 시작 각도 손으로 넣어주기 !*/
    double eastReferenceAngle = +60; // 현재 정동쪽을 바라볼 때 측정된 각도
    double calibratedAngle = measuredAngle - eastReferenceAngle; // 보정 적용
    
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

// 첫번째 GPS에서 얻은 데이터를 처리
void navfix_gps1_Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)  
{// front GPS
	lat1 = gps_msg->latitude;
    lon1 = gps_msg->longitude;
    alt1 = gps_msg->altitude;
    fix1 = gps_msg->status.status;     
   
    double east, north;
    int zone;
    
    if (gps_msg->status.status == -1) 
    {
        ROS_DEBUG_THROTTLE(60, "No fix.");
        return;
    }
	else if(gps_msg->status.status == (0 || 1 || 2))
	{
		ROS_INFO("GPS1 FIX 완료!");
	}
	
    zone = lon1 / 6 + 31;
    
    wgs2utm(lat1, lon1, zone, east, north);

    double easting = east * 0.9996 + 500000;
    double northing = north * 0.9996;
    ROS_INFO("Lat: %f, Lon: %f, Alt: %f", lat1, lon1, alt1);
    ROS_INFO("hi : %f %f ", easting, northing);
    
    Pose1.x = easting;
    Pose1.y = northing;// 첫번째 GPS에서 얻은 데이터를 처리

}




// 두번째 GPS에서 얻은 데이터를 처리
void navfix_gps2_Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)   // rear GPS
{
	lat2 = gps_msg->latitude;
    lon2 = gps_msg->longitude;
    alt2 = gps_msg->altitude;
    fix2 = gps_msg->status.status; 
    
    double  east, north;
    int zone;
    
    if (gps_msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
    {
        ROS_DEBUG_THROTTLE(60,"No fix.");
        return;
    }
    else if(gps_msg->status.status == 1) 
    {
		
		ROS_DEBUG_THROTTLE(60,"GPS2  float.");
		return;
	}
	else if(gps_msg->status.status == 2) 
	{
		ROS_INFO("GPS2  FIX.");
	}
    
    if(lon2 < 0){
        zone = (lon2 + 180) / 6 + 1;
    }
    else{
        zone = lon2 / 6 + 31;
    }
    wgs2utm(lat2, lon2, zone, east, north);

    double easting = east * 0.9996 + 500000;
    double northing = north * 0.9996;
    
    //printf("GPS2 utm[E :%15.9lf    | N :%15.9lf\n",easting, northing);

    Pose2.x = easting;
    Pose2.y = northing;
    //f_x = Pose1.x - Pose2.x;
	diff_y = Pose1.y - Pose2.y;
    /*
    std::cout << std::setprecision(9);
    std::cout << "lat : " << lat << std::endl;
    std::cout << "lon : " << lon << std::endl;
    std::cout << "zone : " << zone << std::endl;
    std::cout << "E : " << easting << std::endl;
    std::cout << "N : " << northing << std::endl;
    */
} 

// IMU Heading값 콜백
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    double roll, pitch, yaw;

  /*
   *   ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    */        

    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);     
            
    m.getRPY(roll, pitch, yaw);
    imu_yaw = yaw;

    //  roll_d  = RAD2DEG(roll);
    //  pitch_d = RAD2DEG(pitch);
    //  yaw_d   = RAD2DEG(yaw);        
            
}

// ----------------------------------------------------------------
int main(int argc, char **argv){
  char buf[2];
  double h_angle = 0.0;
  double utm_x = 0.0;
  double utm_y = 0.0;
 
  /*frame id*/
  std::string odom_frame_id = "odom";
  std::string odom_child_frame_id = "gps1_footprint";
  std::string imu_topic = "/vectornav/IMU";
  
  ros::init(argc, argv, "dual_gps_package");
  ros::NodeHandle n;
  
  ros::param::get("~use_imu_yaw_angle",  use_imu_yaw_angle); //imu의 yaw anlge을 사용
  ros::param::get("~use_two_gps" , use_two_gps); //2개의 GPS를 사용할 경우 yaw값은 2개의 GPS로 부터 계산함   
  ros::param::get("~imu_yaw_offset", imu_yaw_offset);
  ros::param::get("~odom_frame_id", odom_frame_id);
  ros::param::get("~odom_child_frame_id", odom_child_frame_id);  
  ros::param::get("~imu_topic", imu_topic);     
  
   
  ros::Subscriber sub_navsat_fix1 = n.subscribe("/ublox_gps/fix",1,&navfix_gps1_Callback);  // front gps 
  ros::Subscriber subIMU = n.subscribe(imu_topic, 20, &imuCallback);  // imu
  //ros::Subscriber sub_navsat_fix2 = n.subscribe("/gps/fix2",1,&navfix_gps2_Callback);  // rear  gps    
  
  ros::Publisher angle_pub = n.advertise<std_msgs::Float32>("/gps_heading_angle",1);     
  ros::Publisher utm1_pub              = n.advertise<geometry_msgs::Pose2D>("/gps/utm_pos1",1);   
  //ros::Publisher utm2_pub              = n.advertise<geometry_msgs::Pose2D>("/gps/utm_pos2",1); 
  ros::Publisher gps1_fix_status_pub   = n.advertise<std_msgs::Bool>("/gps/fix_status1",1); 
  //ros::Publisher gps2_fix_status_pub   = n.advertise<std_msgs::Bool>("/gps/fix_status2",1); 
  ros::Publisher east_angle_pub        = n.advertise<std_msgs::Float32>("/east_heading_angle",1); 
  

  ros::Publisher gps_odom_pub = n.advertise<nav_msgs::Odometry>("/nav_odom",1); 
  
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
  
  while (ros::ok())
  {	
	
	diff_x = Pose1.x - Pose2.x;
	diff_y = Pose1.y - Pose2.y;
	
	ROS_INFO("%d, %d", fix1, fix2);
	if(fix1 != -1) // /ublox_gps is fixed
	{
	    current_time = ros::Time::now();	
        printf(" %d   %d \n\n\n", use_imu_yaw_angle,  use_two_gps);

        // GPS 1개 + IMU 1개
        if(use_imu_yaw_angle == 1){
            h_angle = imu_yaw + DEG2RAD(imu_yaw_offset);
        }		   

        Pose1.theta = Pose2.theta = h_angle;
        gps_heading_angle.data = h_angle;
        angle_pub.publish(gps_heading_angle);

        double calibratedAngle = calibrateHeadingAngle(RAD2DEG(h_angle));
        east_angle.data = calibratedAngle;
        east_angle_pub.publish(east_angle);

        //Pos -> Ros Coordinate
        ROS_INFO("Front GPS E: %.7lf N: %.7lf",Pose1.x,Pose1.y);
        //if(use_two_gps == 1)      //ROS_INFO("Rear  GPS E: %.7lf N: %.7lf",Pose1.x,Pose1.y);
        ROS_INFO("Heading Angle : %.7lf",RAD2DEG(h_angle));      
        ROS_INFO("Heading Angle(EAST 0) : %.7lf",calibratedAngle); 
        utm1.x = Pose1.x; 	utm1.y = Pose1.y;	utm1.theta = DEG2RAD(calibratedAngle);
        utm2.x = Pose2.x; 	utm2.y = Pose2.y;	utm2.theta = h_angle;

        utm1_pub.publish(utm1);
        // utm2_pub.publish(utm2);
        
        gps_odom1.header.stamp = current_time;
        gps_odom1.header.frame_id = odom_frame_id;
        gps_odom1.child_frame_id = odom_child_frame_id;
    
        //odom_oriention trans to odom_quat_temp
        odom_quat = tf::createQuaternionMsgFromYaw(h_angle);//yaw trans quat
        
        gps_odom1.pose.pose.position.x, gps_odom1.pose.pose.position.y = lon1, lat1;
        gps_odom1.pose.pose.position.z = 0 ;
        gps_odom1.pose.pose.orientation = odom_quat;

        gps_odom_pub.publish(gps_odom1);
        
        std_msgs::Bool gps_fix_status1, gps_fix_status2;
        
        gps_fix_status1.data = gps_fix_status2.data = false;
        
        if(fix1 != -1){ // <if /ublox_gps is fixed>
            gps_fix_status1.data = true;
        } 
        gps1_fix_status_pub.publish(gps_fix_status1);
        
        
        if(fix2==2){
            gps_fix_status2.data = true;
        } 
        // gps2_fix_status_pub.publish(gps_fix_status2);
        
    
    }  
	
	
	loop_rate.sleep();
    ros::spinOnce();
   
  }
  return 0;
}