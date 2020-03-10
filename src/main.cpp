#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <cmath>

using namespace std;

bool receivedSetPoint = false;

ros::Publisher publisherError;

tf2::Quaternion setpointOrientation;

void setpointCallback(const geometry_msgs::Pose::ConstPtr &setpoint)
{
    receivedSetPoint = true;
    fromMsg(setpoint->orientation, setpointOrientation);
}

void measurementCallback(const geometry_msgs::Pose::ConstPtr &measurement)
{
    if (receivedSetPoint == false)
        return;

    tf2::Quaternion measurementOrientation;
    fromMsg(measurement->orientation, measurementOrientation);

    tf2::Quaternion error = setpointOrientation * measurementOrientation.inverse();
    tf2::Vector3 v(1.0f, 0, 0);

    v = quatRotate(error, v);

    float angleError = atan2(v.z(), v.x());

    angleError *= 180 / M_PI;
    std_msgs::Float64 errorMsg;
    errorMsg.data = (double)angleError;
    publisherError.publish(errorMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "error");
    ros::NodeHandle nodeHandle;

    ros::Subscriber subscriberSetPoint = nodeHandle.subscribe("setpoint", 1000, setpointCallback);
    ros::Subscriber subscriberMeasurement = nodeHandle.subscribe("measurement", 1000, measurementCallback);

    publisherError = nodeHandle.advertise<std_msgs::Float64>("error", 1000);

    ros::spin();

    return 0;
}
