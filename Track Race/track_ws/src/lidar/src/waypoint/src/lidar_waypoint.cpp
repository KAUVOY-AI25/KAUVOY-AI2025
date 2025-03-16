/*
 * written by. 민환
 * Recently Updated on 2025-02-13
 * Delaunay 삼각분할 알고리즘 이용
 * 삼각형 중심을 계산하여 Waypoint 도출
 * lidar/scripts에 있는 tf_broadcaster.py 실행시켜야함
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 Point;

ros::Publisher pub_waypoints;
ros::Publisher pub_edges;
ros::Publisher pub_path;
tf::TransformListener* tf_listener;

void delaunay_callback(const visualization_msgs::MarkerArray::ConstPtr& spheres) {
    std::vector<Point> cluster_centers;
    std::vector<geometry_msgs::Point> waypoints;
    visualization_msgs::Marker waypoint_marker, edges, path;

    for (const auto& marker : spheres->markers) {
        geometry_msgs::PointStamped input_point, transformed_point;
        input_point.header.frame_id = "velodyne";
        input_point.header.stamp = ros::Time(0);
        input_point.point = marker.pose.position;

        try {
            tf_listener->transformPoint("map", input_point, transformed_point);
            cluster_centers.emplace_back(transformed_point.point.x, transformed_point.point.y);
        } catch (tf::TransformException &ex) {
            ROS_WARN("TF Transform failed: %s", ex.what());
            return;
        }
    }

    Delaunay dt;
    dt.insert(cluster_centers.begin(), cluster_centers.end());

    for (auto it = dt.finite_faces_begin(); it != dt.finite_faces_end(); ++it) {
        auto v0 = it->vertex(0)->point();
        auto v1 = it->vertex(1)->point();
        auto v2 = it->vertex(2)->point();

        geometry_msgs::Point waypoint;
        waypoint.x = (v0.x() + v1.x() + v2.x()) / 3.0;
        waypoint.y = (v0.y() + v1.y() + v2.y()) / 3.0;
        waypoint.z = 0.0;
        waypoints.push_back(waypoint);
    }

    waypoint_marker.header.frame_id = "map";
    waypoint_marker.ns = "waypoints";
    waypoint_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    waypoint_marker.scale.x = 0.3;
    waypoint_marker.scale.y = 0.3;
    waypoint_marker.scale.z = 0.3;
    waypoint_marker.color.r = 1.0;
    waypoint_marker.color.g = 0.0;
    waypoint_marker.color.b = 0.0;
    waypoint_marker.color.a = 1.0;
    waypoint_marker.points = waypoints;

    edges.header.frame_id = "map";
    edges.ns = "waypoint_edges";
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.scale.x = 0.05;
    edges.color.r = 0.0;
    edges.color.g = 0.0;
    edges.color.b = 1.0;
    edges.color.a = 1.0;

    path.header.frame_id = "map";
    path.ns = "waypoint_path";
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.scale.x = 0.1;
    path.color.r = 0.0;
    path.color.g = 1.0;
    path.color.b = 0.0;
    path.color.a = 1.0;
    path.points = waypoints;

    for (auto it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
        auto p1 = it->first->vertex(it->second)->point();
        auto p2 = it->first->vertex(2)->point();
        geometry_msgs::Point pt1, pt2;
        pt1.x = p1.x(); pt1.y = p1.y();
        pt2.x = p2.x(); pt2.y = p2.y();
        edges.points.push_back(pt1);
        edges.points.push_back(pt2);
    }

    pub_waypoints.publish(waypoint_marker);
    pub_edges.publish(edges);
    pub_path.publish(path);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "delaunay_waypoint_generator");
    ros::NodeHandle nh;
    tf_listener = new tf::TransformListener;

    ros::Subscriber sub = nh.subscribe("cluster_spheres", 1, delaunay_callback);
    pub_waypoints = nh.advertise<visualization_msgs::Marker>("waypoints", 1);
    pub_edges = nh.advertise<visualization_msgs::Marker>("waypoint_edges", 1);
    pub_path = nh.advertise<visualization_msgs::Marker>("waypoint_path", 1);

    ROS_INFO("Delaunay Waypoint Generator node started.");
    ros::spin();

    delete tf_listener;
    return 0;
}