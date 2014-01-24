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

double ds = 0.1;
double di = 0.2;
std::string frame_id;
int pose_array_counter = 1;
const int max_size = 100;

geometry_msgs::PoseArray pose_array;

void createMarker(float x, float y, float z, float radius, float r, float g,
		float b, float a, visualization_msgs::Marker& marker) {
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "/";
	marker.id = 0 + int(a * 100);
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

void updateMarkerArray(const geometry_msgs::TransformStampedConstPtr& transform, boost::shared_ptr<visualization_msgs::MarkerArray> array) {
//	visualization_msgs::MarkerArray array;

	visualization_msgs::Marker innerBall;
	visualization_msgs::Marker outerBall;

	createMarker(transform->transform.translation.x, transform->transform.translation.y,
			transform->transform.translation.z, di, 0.0, 0.0, 0.0, 0.5, innerBall);
	createMarker(transform->transform.translation.x, transform->transform.translation.y,
			transform->transform.translation.z, ds, 1.0, 1.0, 0.0, 0.3, outerBall);

	array->markers.push_back(innerBall);
	array->markers.push_back(outerBall);
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

void syncCallback(const geometry_msgs::Vector3StampedConstPtr& unitVec,
		const geometry_msgs::TransformStampedConstPtr& pos, boost::shared_ptr<geometry_msgs::PoseArray> pose_array) {
	//ROS_INFO("synced callback");

	geometry_msgs::Pose pose;
	pose.position.x = pos->transform.translation.x;
	pose.position.y = pos->transform.translation.y;
	pose.position.z = pos->transform.translation.z;
	double roll = 0;
    double pitch = asin(unitVec->vector.z);
    //double yaw = acos(unitVec->vector.x / cos(pitch));

	//ROS_INFO("vector.z %f", unitVec->vector.z );
	//ROS_INFO("pitch value %f", pitch);
	//ROS_INFO("cos of pitch %f", cos(pitch));
	//ROS_INFO("yaw value %f", yaw);


    double sign;
    if (unitVec->vector.z >=0){
        sign= 0;
    }else{
        sign = M_PI;
    }


    tf::Quaternion q = tf::createQuaternionFromRPY(roll,sign+pitch, sign);
//    tf::Quaternion q = tf::createQuaternionFromYaw(pitch);
    pose.orientation.w = q.getW();
    pose.orientation.x = q.getX();
    pose.orientation.y = q.getY();
    pose.orientation.z = q.getZ();

    //ROS_INFO("pose orientation : %f, %f,  %f, %f", q.getW(), q.getX(), q.getY(), q.getZ());


	signed int index = pose_array_counter%max_size;
	//ROS_INFO("index: %i", index);
	if (pose_array_counter <= max_size){
		pose_array->poses.push_back(pose);
	}
	else{
		pose_array->poses[index ] = pose;
	}
	pose_array_counter++;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rviz_marker_listener");

	ros::NodeHandle node_handle("~");
	node_handle.param<std::string>("frameid", frame_id, "floor_link");
	ROS_INFO("setting frame id to %s", frame_id.c_str());

	ros::Publisher vis_pub = node_handle.advertise<
			visualization_msgs::MarkerArray>("collision_array", 100);

	ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseArray>("pose_array", 100);

	boost::shared_ptr<visualization_msgs::MarkerArray> marker_array(new visualization_msgs::MarkerArray);
	ros::Subscriber vis_sub = node_handle.subscribe<geometry_msgs::TransformStamped>(
			"position2", 100, boost::bind(&updateMarkerArray, _1, marker_array));


	ros::Subscriber ds_sub = node_handle.subscribe<std_msgs::Float64>("ds", 1,
			updateDS);
	ros::Subscriber di_sub = node_handle.subscribe<std_msgs::Float64>("di", 1,
			updateDI);


	boost::shared_ptr<geometry_msgs::PoseArray> pose_array(new geometry_msgs::PoseArray);
	pose_array->header.frame_id = frame_id;
	pose_array->header.stamp = ros::Time();
	message_filters::Subscriber<geometry_msgs::Vector3Stamped> unit_sub(
			node_handle, "unitvec", 1);
	message_filters::Subscriber<geometry_msgs::TransformStamped> pos_sub(
			node_handle, "position1", 1);
	typedef message_filters::sync_policies::ApproximateTime<
			geometry_msgs::Vector3Stamped, geometry_msgs::TransformStamped> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(2),
			unit_sub, pos_sub);
	sync.registerCallback(boost::bind(&syncCallback, _1, _2, pose_array));

	ros::Rate r(10);

	while(node_handle.ok()){

		pose_pub.publish(pose_array);
		vis_pub.publish(marker_array);
		marker_array->markers.clear();
		ros::spinOnce();
		r.sleep();
	}

}
