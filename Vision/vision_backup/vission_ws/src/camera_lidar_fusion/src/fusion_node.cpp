#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h> 
#include <opencv2/opencv.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <std_msgs/Float32MultiArray.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class FusionNode {
public:
    FusionNode() : extrinsic_matrix_(Eigen::Matrix4d::Identity()), intrinsic_matrix_(Eigen::Matrix3d::Identity()) {
        //camera_sub_ = nh_.subscribe("/camera1/usb_cam1/image_raw", 1, &FusionNode::cameraCallback, this);
        // 카메라 토픽을 panorama_node의 출력으로 수정
	camera_sub_ = nh_.subscribe("/panorama/image", 1, &FusionNode::cameraCallback, this);

        marker_sub_ = nh_.subscribe("/visualization_marker_array", 1, &FusionNode::markerCallback, this);
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera_lidar_fusion/output", 1);
        bounding_box_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/bounding_box_points", 1);

        intrinsic_matrix_ << 440.448278884377, 0, 321.492290400808,
                             0, 455.317535181246, 135.774750100701,
                             0, 0, 1;

        extrinsic_matrix_ << -0.0405243758643159, -0.999178255809739, 0.000766862318099915, -0.0201615025392103,
                             -0.431570238357808, 0.0168112786088913, -0.901922674221753, 0.536260686706105,
                              0.901168632568063, -0.0368808084041212, -0.431896864594854, 1.23601061670107,
                              0, 0, 0, 1;
    }

    void cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            current_image_ = cv_ptr->image;
            current_image_msg_ = msg;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& marker_array) {
        ROS_INFO("Marker callback triggered");
        if (current_image_.empty()) {
            ROS_WARN("No image received yet.");
            return;
        }

        cv::Mat overlay_image = current_image_.clone();
        std_msgs::Float32MultiArray points_array;

        for (const auto& marker : marker_array->markers) {
            Eigen::Vector3d position(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
            
            double x = position[0];
            double y = position[1];
            double z = position[2];

            // Calculate the distance of the object
            double distance = position.norm();
            ROS_INFO("Object coordinates: x = %f, y = %f, z = %f", x, y, z);
            ROS_INFO("Object distance: %f meters", distance);

            // Project 3D coordinates to 2D image plane
            Eigen::Vector2d pixel_coords = projectTo2D(position);

            // Draw a rectangle on the image
            cv::rectangle(overlay_image, cv::Point(pixel_coords[0] - 5, pixel_coords[1] - 5),
                          cv::Point(pixel_coords[0] + 5, pixel_coords[1] + 5), cv::Scalar(0, 255, 0), 2);

            // Display the distance and coordinates on the image
            std::string distance_text = cv::format("Distance: %.2f m", distance);
            std::string x_text = cv::format("x: %.2f", x);
            std::string y_text = cv::format("y: %.2f", y);
            std::string z_text = cv::format("z: %.2f", z);

            int baseline = 0;
            cv::Size textSize = cv::getTextSize(distance_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
            cv::Point text_origin(pixel_coords[0] + 10, pixel_coords[1]);

            cv::putText(overlay_image, distance_text, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            cv::putText(overlay_image, x_text, text_origin + cv::Point(0, textSize.height + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            cv::putText(overlay_image, y_text, text_origin + cv::Point(0, 2 * (textSize.height + 5)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            cv::putText(overlay_image, z_text, text_origin + cv::Point(0, 3 * (textSize.height + 5)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

            // Calculate the bounding box points (assuming a simple square around the marker)
            points_array.data.clear();
            double box_size = 0.5;  // Example size
            points_array.data.push_back(marker.pose.position.x - box_size);
            points_array.data.push_back(marker.pose.position.y - box_size);
            points_array.data.push_back(marker.pose.position.x + box_size);
            points_array.data.push_back(marker.pose.position.y + box_size);

            // Publish the bounding box points
            bounding_box_pub_.publish(points_array);
        }

        cv_bridge::CvImage out_msg;
        out_msg.header = current_image_msg_->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = overlay_image;
        image_pub_.publish(out_msg.toImageMsg());
    }

private:
    Eigen::Vector2d projectTo2D(const Eigen::Vector3d& pt_3d) {
        Eigen::Vector4d pt_3d_homogeneous(pt_3d[0], pt_3d[1], pt_3d[2], 1.0);
        Eigen::Vector4d pt_camera_frame = extrinsic_matrix_ * pt_3d_homogeneous;
        Eigen::Vector3d pt_2d_homogeneous = intrinsic_matrix_ * pt_camera_frame.head<3>();

        Eigen::Vector2d pt_2d;
        pt_2d[0] = pt_2d_homogeneous[0] / pt_2d_homogeneous[2];
        pt_2d[1] = pt_2d_homogeneous[1] / pt_2d_homogeneous[2];

        return pt_2d;
    }

    ros::NodeHandle nh_;
    ros::Subscriber camera_sub_;
    ros::Subscriber marker_sub_;
    ros::Publisher image_pub_;
    ros::Publisher bounding_box_pub_;
    cv::Mat current_image_;
    sensor_msgs::ImageConstPtr current_image_msg_;
    Eigen::Matrix4d extrinsic_matrix_;
    Eigen::Matrix3d intrinsic_matrix_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion_node");
    FusionNode fusion_node;
    ros::spin();
    return 0;
}

