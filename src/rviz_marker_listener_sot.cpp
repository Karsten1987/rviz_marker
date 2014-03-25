/*
 * rviz_marker_listener.cpp
 *
 *  Created on: 2 Sep 2013
 *      Author: Karsten Knese
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "boost/bind.hpp"
#include "math.h"
#include <std_msgs/Float64.h>

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>

const std::string FRAME_ID                  = "floor_link";
const std::string DS_BASE_TOPIC             = "/sot_controller/ds_";
const std::string DI_BASE_TOPIC             = "/sot_controller/di_";
const std::string TRANSFORM_BASE_TOPIC      = "/sot_controller/p2_";

typedef visualization_msgs::MarkerArray     MArray;
typedef visualization_msgs::Marker          M;
typedef geometry_msgs::TransformStamped     Transform;


double ds                                   = 0.1;
double di                                   = 0.2;

void createMarker(float x, float y, float z, float radius, float r, float g,
        float b, float a, M& marker) {
    marker.header.frame_id = FRAME_ID;
	marker.header.stamp = ros::Time();
	marker.ns = "/";
	marker.id = 0 + int(a * 100);
    marker.type = M::SPHERE;
    marker.action = M::ADD;
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

void updateMarkerArray(const Transform::ConstPtr& transform, MArray& array)
{
    M innerBall;
    M outerBall;

    createMarker(transform->transform.translation.x, transform->transform.translation.y,
            transform->transform.translation.z, di, 0.0, 0.0, 0.0, 0.5, innerBall);
    createMarker(transform->transform.translation.x, transform->transform.translation.y,
            transform->transform.translation.z, ds, 1.0, 1.0, 0.0, 0.3, outerBall);

    //array.markers[0] = innerBall;
    array.markers[1] = outerBall;
}

void updateDS(const std_msgs::Float64ConstPtr& newDS) {
	if (newDS->data != ds) {
		ds = newDS->data;
	}
}

void updateDI(const std_msgs::Float64ConstPtr& newDI) {
	if (newDI->data != di) {
		di = newDI->data;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rviz_marker");

    ros::NodeHandle node_handle("~");

    const std::string actuated_joint_signal = argv[1];

    ros::Publisher vis_pub = node_handle.advertise<MArray>("external_collision", 100);


    MArray m_array;
    m_array.markers.resize(2);

    const std::string transform_topic = TRANSFORM_BASE_TOPIC+actuated_joint_signal;
    ros::Subscriber vis_sub = node_handle.subscribe<Transform>(
            transform_topic, 100, boost::bind(&updateMarkerArray, _1, boost::ref(m_array)));
    ROS_INFO_STREAM("listening to transform topic\t" << transform_topic);


    const std::string ds_topic = DS_BASE_TOPIC+actuated_joint_signal;
    ros::Subscriber ds_sub = node_handle.subscribe<std_msgs::Float64>(ds_topic, 1,
			updateDS);
    ROS_INFO_STREAM("listening to ds topic\t" << ds_topic);

    const std::string di_topic = DI_BASE_TOPIC+actuated_joint_signal;
    ros::Subscriber di_sub = node_handle.subscribe<std_msgs::Float64>(di_topic, 1,
			updateDI);
    ROS_INFO_STREAM("listening to di topic\t" << di_topic);


    ros::Rate r(100);
	while(node_handle.ok()){
        vis_pub.publish(m_array);
		ros::spinOnce();
		r.sleep();
	}

}
