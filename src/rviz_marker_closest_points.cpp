#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "boost/bind.hpp"
#include "math.h"
#include <std_msgs/Float64.h>

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"

std::string frame_id = "floor_link";
double ds = 0.03;
double di = 0.1;

void updateDS(const std_msgs::Float64ConstPtr& newDS) {
    if (newDS->data != ds) {
        ds = newDS->data;
        ROS_INFO_STREAM("new ds value set to: " << ds);
    }
}

void updateDI(const std_msgs::Float64ConstPtr& newDI) {
    if (newDI->data != di) {
        di = newDI->data;
        ROS_INFO_STREAM("new di value set to: " << di);
    }
}


void createMarker(float x, float y, float z, float radius, float r, float g,
                  float b, float a, visualization_msgs::Marker& marker) {
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "/";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = radius * 2;
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
}

void updateMarker(const geometry_msgs::Vector3& transform, visualization_msgs::Marker& marker){
    createMarker(transform.x, transform.y,
                 transform.z, 0.01, 0.0, 1.0, 0.0, 1, marker);
}

void updateMarkerArray(const  geometry_msgs::TransformConstPtr& transform,
                       const int& idx,
                       const int& i,
                       const int& j,
                       boost::shared_ptr<visualization_msgs::MarkerArray>& collision_points_marker)
{
    visualization_msgs::Marker marker;
    updateMarker(transform->translation, marker);
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.id = idx;
    collision_points_marker->markers[idx] = marker;
}


void updateCollisionAvoidance(const  geometry_msgs::TransformConstPtr& transform,
                              const int& idx,
                              const int& i,
                              const int& j,
                              boost::shared_ptr<visualization_msgs::MarkerArray>& collision_points_marker)
{
    visualization_msgs::Marker marker;
    updateMarker(transform->translation, marker);
    marker.id = idx+100;
    marker.scale.x = ds*2;
    marker.scale.y = ds*2;
    marker.scale.z = ds*2;
    marker.color.a = 0.2;
    collision_points_marker->markers[idx] = marker;
}

void fillLineMarker(const boost::shared_ptr<visualization_msgs::MarkerArray>& collision_points_p1,
                    const boost::shared_ptr<visualization_msgs::MarkerArray>& collision_points_p2,
                    boost::shared_ptr<visualization_msgs::MarkerArray>& collision_points_line){

    for (int i =0; i< collision_points_p1->markers.size(); ++i){

            int idx = i*collision_points_p1->markers.size()+ i;

                visualization_msgs::Marker marker1 = collision_points_p1->markers[i];
                visualization_msgs::Marker marker2 = collision_points_p2->markers[i];

                geometry_msgs::Point p1;
                p1.x = marker1.pose.position.x;
                p1.y = marker1.pose.position.y;
                p1.z = marker1.pose.position.z;

                geometry_msgs::Point p2;
                p2.x = marker2.pose.position.x;
                p2.y = marker2.pose.position.y;
                p2.z = marker2.pose.position.z;


                if (p2.x != 0 && p2.y != 0 && p2.x != 0 &&
                    p1.x != 0 && p1.y != 0 && p1.x != 0)
                {
                    visualization_msgs::Marker line;
                    line.header.frame_id = frame_id;
                    line.header.stamp = ros::Time();
                    line.ns = "/";
                    line.type = visualization_msgs::Marker::LINE_STRIP;
                    line.action = visualization_msgs::Marker::ADD;
                    line.id = idx;

                    line.scale.x = 0.005;
                    line.color.r = 0.5;
                    line.color.g = 0.5;
                    line.color.b = 0.5;
                    line.color.a = 1.0;

                    line.points.push_back(p1);
                    line.points.push_back(p2);

                    collision_points_line->markers.push_back(line);
                }
    }

}

void split(std::vector<std::string> &tokens, const std::string &text, char sep) {
    int start = 0, end = 0;
    while ((end = text.find(sep, start)) != std::string::npos) {
        tokens.push_back(text.substr(start, end - start));
        start = end + 1;
    }
    tokens.push_back(text.substr(start));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_marker_closest_points");

    std::vector<std::string> collision_objects;
    split(collision_objects, argv[1], ':');

    ros::NodeHandle node_handle("~");
    node_handle.param<std::string>("frameid", frame_id, "floor_link");
    ROS_INFO("setting frame id to %s", frame_id.c_str());

    boost::shared_ptr<visualization_msgs::MarkerArray> collision_points_p1
            = boost::shared_ptr<visualization_msgs::MarkerArray>(new visualization_msgs::MarkerArray());

    boost::shared_ptr<visualization_msgs::MarkerArray> collision_points_p2
            = boost::shared_ptr<visualization_msgs::MarkerArray>(new visualization_msgs::MarkerArray());

    boost::shared_ptr<visualization_msgs::MarkerArray> collision_points_line
            = boost::shared_ptr<visualization_msgs::MarkerArray>(new visualization_msgs::MarkerArray());

    boost::shared_ptr<visualization_msgs::MarkerArray> avoidance_marker
            = boost::shared_ptr<visualization_msgs::MarkerArray>(new visualization_msgs::MarkerArray());

    std::vector<ros::Subscriber> point_listener;
    std::vector<ros::Subscriber> avoidance_listener;

    avoidance_marker->markers.resize(collision_objects.size()*collision_objects.size());
    collision_points_p1->markers.resize(collision_objects.size()*collision_objects.size());
    collision_points_p2->markers.resize(collision_objects.size()*collision_objects.size());
//    collision_points_line->markers.resize(collision_objects.size()*collision_objects.size());

    for (int i = 0; i < collision_objects.size(); ++i) {
        for (int j = 0; j < collision_objects.size(); ++j) {
            int idx = i*collision_objects.size()+j;

            // for closest points
            ros::Subscriber listener_p1 = node_handle.subscribe<geometry_msgs::Transform>
                    ("p1_"+collision_objects[i]+collision_objects[j], 100, boost::bind(&updateMarkerArray, _1, idx, i,j,collision_points_p1));
            point_listener.push_back(listener_p1);

            ros::Subscriber listener_p2 = node_handle.subscribe<geometry_msgs::Transform>
                    ("p2_"+collision_objects[i]+collision_objects[j], 100, boost::bind(&updateMarkerArray, _1, idx, i,j,collision_points_p2));
            point_listener.push_back(listener_p2);

            // for collision avoidance (namely P2, considered as fixed point)
            ros::Subscriber avoidance = node_handle.subscribe<geometry_msgs::Transform>
                    ("avoid_"+collision_objects[i]+collision_objects[j], 100, boost::bind(&updateCollisionAvoidance, _1, idx, i,j,avoidance_marker));
            avoidance_listener.push_back(avoidance);
        }
    }

    ros::Subscriber ds_sub = node_handle.subscribe<std_msgs::Float64>("ds", 1,
                                                                      updateDS);
    ros::Subscriber di_sub = node_handle.subscribe<std_msgs::Float64>("di", 1,
                                                                      updateDI);

    ros::Publisher pub_p1 = node_handle.advertise<visualization_msgs::MarkerArray>("closest_points_p1", 100);
    ros::Publisher pub_p2 = node_handle.advertise<visualization_msgs::MarkerArray>("closest_points_p2", 100);
    ros::Publisher avoidance_pub = node_handle.advertise<visualization_msgs::MarkerArray>("avoidance", 100);
    ros::Publisher pub_line = node_handle.advertise<visualization_msgs::MarkerArray>("closest_points_lines", 100);

    ros::Rate r(100);

    while(node_handle.ok()){

        pub_p1.publish(*collision_points_p1);
        pub_p2.publish(*collision_points_p2);
        avoidance_pub.publish(*avoidance_marker);

        fillLineMarker(collision_points_p1,collision_points_p2,collision_points_line);
        pub_line.publish(collision_points_line);

        ros::spinOnce();
        r.sleep();
        collision_points_line->markers.clear();
    }

}
