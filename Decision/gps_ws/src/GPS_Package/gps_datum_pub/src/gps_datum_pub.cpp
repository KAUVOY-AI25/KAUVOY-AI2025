  #include <stdio.h>
  #include <stdlib.h>

  #include "ros/ros.h"
  #include "geometry_msgs/Vector3.h"
  #include <XmlRpcException.h>

  // Datum parameter - required
  double datum_lat; 
  double datum_lon;
  double datum_yaw;
    
    
  int main(int argc, char **argv){
      
    ros::init(argc, argv, "GPS_Datum_Publisher");

    ros::NodeHandle n;
    ros::NodeHandle nh_priv("~");
      
    ros::Publisher pub = n.advertise<geometry_msgs::Vector3>("/gps/datum", 10);
      
    if(!nh_priv.hasParam("datum") ){
      ROS_FATAL("private <datum> parameter is not supplied in "  "geonav_transform configuration");
      exit(1);
    } 

    // datum 파라미터 값을 가져오기 위한 XmlRpcValue 객체 선언 및 파라미터 개수 검증
    XmlRpc::XmlRpcValue datum_config;    
    nh_priv.getParam("datum", datum_config);
      
    ROS_ASSERT(datum_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(datum_config.size() >= 3);
        
    if (datum_config.size() > 3){
      ROS_WARN_STREAM("Deprecated datum parameter configuration detected. "
      "Only the first three parameters "
      "(latitude, longitude, yaw) will be used. frame_ids "
      "will be derived from odometry and navsat inputs.");
    }

    // Launch파일에 있는 Parameter를 Parsing해서 저장
    std::ostringstream ostr;
    ostr << datum_config[0] << " " << datum_config[1] << " " << datum_config[2];
    std::istringstream istr(ostr.str());
    istr >> datum_lat >> datum_lon >> datum_yaw;
    
    ROS_INFO("GPS Datum [%12.9lf %12.9lf %12.9lf]", datum_lat, datum_lon, datum_yaw);
    
    ros::Rate loop_rate(10);  // 10hz

    while (ros::ok()){
      
      // 메시지 객체 생성 및 초기화
      geometry_msgs::Vector3 GPS_Datum;
      
      GPS_Datum.x = datum_lat;
      GPS_Datum.y = datum_lon;
      GPS_Datum.z = datum_yaw;
    
      pub.publish(GPS_Datum);

      ROS_INFO("GPS Datum [%12.9lf %12.9lf %12.9lf]", datum_lat, datum_lon, datum_yaw);

      loop_rate.sleep();

      ros::spinOnce();
    }
    
  }
