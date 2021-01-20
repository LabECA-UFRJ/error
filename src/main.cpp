#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Float64.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>

#include <cmath>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "error");
	ros::NodeHandle nodeHandle;

	ros::Publisher publisherError = nodeHandle.advertise<std_msgs::Float64>("error", 1000);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	std::string target;
	if (!ros::param::get("~target_frame", target)) {
		ROS_ERROR("target_frame param not set");
		return 0;
	}

	std::string source;
	if (!ros::param::get("~source_frame", source)) {
		ROS_ERROR("source_frame param not set");
		return 0;
	}
	
	ros::Rate rate(60.0);
	while (nodeHandle.ok())
	{
		if (!tfBuffer.canTransform(target, source, ros::Time(0), ros::Duration(1.0))) {
			ros::Duration(1.0).sleep();
			continue;
		}

		geometry_msgs::TransformStamped transformStamped;

		try {
			transformStamped = tfBuffer.lookupTransform(target, source, ros::Time(0));
		} catch (tf2::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			break;
		}

		tf2::Quaternion q(
			transformStamped.transform.rotation.x, 
			transformStamped.transform.rotation.y, 
			transformStamped.transform.rotation.z, 
			transformStamped.transform.rotation.w
			);

		tf2::Vector3 v(1.0f, 0, 0);

		v = quatRotate(q, v);

		double angleError = atan2(v.y(), v.x());
		angleError *= 180 / M_PI;
		
		std_msgs::Float64 errorMsg;
		errorMsg.data = angleError;
		publisherError.publish(errorMsg);

		rate.sleep();
	}

	return 0;
}
