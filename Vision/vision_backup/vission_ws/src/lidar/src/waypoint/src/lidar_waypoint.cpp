#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <vector>
#include <set>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 Point;

typedef std::pair<Point, Point> Edge;

ros::Publisher pub_waypoints;
ros::Publisher pub_edges;
ros::Publisher pub_path;

void delaunay_callback(const visualization_msgs::MarkerArray::ConstPtr& spheres) {
    std::vector<Point> cluster_centers;
    std::set<Edge> internal_edges;
    std::vector<geometry_msgs::Point> waypoints;
    visualization_msgs::Marker edges, path, waypoint_marker;

    for (const auto& marker : spheres->markers) {
        cluster_centers.emplace_back(marker.pose.position.x, marker.pose.position.y);
    }

    Delaunay dt;
    dt.insert(cluster_centers.begin(), cluster_centers.end());

    // 외부 삼각형 제거 (경계 삼각형 제외)
    for (auto it = dt.finite_faces_begin(); it != dt.finite_faces_end(); ++it) {
        if (dt.is_infinite(it)) continue;
        
        auto v0 = it->vertex(0)->point();
        auto v1 = it->vertex(1)->point();
        auto v2 = it->vertex(2)->point();

        internal_edges.insert({std::min(v0, v1), std::max(v0, v1)});
        internal_edges.insert({std::min(v1, v2), std::max(v1, v2)});
        internal_edges.insert({std::min(v2, v0), std::max(v2, v0)});
    }

    // 내부 엣지의 중점을 Waypoint로 생성
    for (const auto& edge : internal_edges) {
        geometry_msgs::Point waypoint;
        waypoint.x = (edge.first.x() + edge.second.x()) / 2.0;
        waypoint.y = (edge.first.y() + edge.second.y()) / 2.0;
        waypoint.z = 0.0;
        waypoints.push_back(waypoint);
    }

    waypoint_marker.header.frame_id = "map";
    waypoint_marker.ns = "waypoints";
    waypoint_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    waypoint_marker.scale.x = 0.2;
    waypoint_marker.scale.y = 0.2;
    waypoint_marker.scale.z = 0.2;
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

    for (const auto& edge : internal_edges) {
        geometry_msgs::Point pt1, pt2;
        pt1.x = edge.first.x();
        pt1.y = edge.first.y();
        pt2.x = edge.second.x();
        pt2.y = edge.second.y();
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

    ros::Subscriber sub = nh.subscribe("cluster_spheres", 1, delaunay_callback);
    pub_waypoints = nh.advertise<visualization_msgs::Marker>("waypoints", 1);
    pub_edges = nh.advertise<visualization_msgs::Marker>("waypoint_edges", 1);
    pub_path = nh.advertise<visualization_msgs::Marker>("waypoint_path", 1);

    ROS_INFO("Delaunay Waypoint Generator node started.");
    ros::spin();

    return 0;
}
