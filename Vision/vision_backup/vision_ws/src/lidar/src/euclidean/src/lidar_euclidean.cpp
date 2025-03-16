/*
 * written by. 민환
 * Recently Updated on 2025-01-17
 * KD 트리 기반 Euclidean Clustering
 * MarkersArray : 1. center sphere / 2. box&ID
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <visualization_msgs/MarkerArray.h>
#include <cstdlib>

ros::Publisher pub_clusters;
ros::Publisher pub_boxes_and_texts;
ros::Publisher pub_spheres;

void clustering_callback(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);

    // KD 트리 기반 Euclidean Clustering
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.2); // 점들 간 거리 허용값
    ec.setMinClusterSize(5);     // 최소 클러스터 크기
    ec.setMaxClusterSize(30);   // 최대 클러스터 크기
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    visualization_msgs::MarkerArray boxes_and_texts;
    visualization_msgs::MarkerArray spheres;
    int cluster_id = 0;

    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::CentroidPoint<pcl::PointXYZI> centroid;

        for (const auto& idx : indices.indices) {
            pcl::PointXYZI point = cloud->points[idx];
            cluster_cloud->points.push_back(point);
            centroid.add(point);

            // 클러스터 점 추가
            point.intensity = static_cast<float>(cluster_id);
            clustered_cloud->points.push_back(point);
        }

        pcl::PointXYZI center;
        centroid.get(center);

        // 사각형 (Bounding Box)
        visualization_msgs::Marker box;
        box.header.frame_id = input->header.frame_id;
        box.header.stamp = ros::Time::now();
        box.ns = "boxes_and_texts";
        box.id = cluster_id;
        box.type = visualization_msgs::Marker::CUBE;
        box.action = visualization_msgs::Marker::ADD;
        box.pose.position.x = center.x;
        box.pose.position.y = center.y;
        box.pose.position.z = center.z;
        box.scale.x = 0.3;
        box.scale.y = 0.3;
        box.scale.z = 0.7;
        box.color.r = 0.0;
        box.color.g = 1.0;
        box.color.b = 0.0;
        box.color.a = 0.5;
        boxes_and_texts.markers.push_back(box);

        // ID 텍스트
        visualization_msgs::Marker text;
        text.header.frame_id = input->header.frame_id;
        text.header.stamp = ros::Time::now();
        text.ns = "boxes_and_texts";
        text.id = cluster_id + 1000; // ID가 겹치지 않도록 조정
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose.position.x = center.x;
        text.pose.position.y = center.y;
        text.pose.position.z = center.z + 1.0; // 텍스트를 상단에 배치
        text.scale.z = 0.5;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        text.text = "ID: " + std::to_string(cluster_id);
        boxes_and_texts.markers.push_back(text);

        // 중심점 원 (SPHERE)
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = input->header.frame_id;
        sphere.header.stamp = ros::Time::now();
        sphere.ns = "spheres";
        sphere.id = cluster_id;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.pose.position.x = center.x;
        sphere.pose.position.y = center.y;
        sphere.pose.position.z = center.z;
        sphere.scale.x = 0.1;
        sphere.scale.y = 0.1;
        sphere.scale.z = 0.1;
        sphere.color.r = 0.0;
        sphere.color.g = 0.0;
        sphere.color.b = 1.0;
        sphere.color.a = 1.0;
        spheres.markers.push_back(sphere);

        cluster_id++;
    }

    // 각각의 MarkerArray 퍼블리시
    pub_boxes_and_texts.publish(boxes_and_texts);
    pub_spheres.publish(spheres);

    // 클러스터링된 포인트 클라우드 퍼블리시
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*clustered_cloud, output);
    output.header = input->header;
    pub_clusters.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "euclidean_clustering");
    ros::NodeHandle nh;

    // 구독 및 퍼블리시
    ros::Subscriber sub = nh.subscribe("lidar_ransac", 1, clustering_callback);
    pub_clusters = nh.advertise<sensor_msgs::PointCloud2>("lidar_euclidean", 1);
    pub_boxes_and_texts = nh.advertise<visualization_msgs::MarkerArray>("cluster_boxes_and_texts", 1);
    pub_spheres = nh.advertise<visualization_msgs::MarkerArray>("cluster_spheres", 1);

    ROS_INFO("Euclidean Clustering node started");
    ros::spin();

    return 0;
}

