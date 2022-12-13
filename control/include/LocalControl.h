#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "lidar_detection/lidar.h"
#include "control/control.h"
#include <iostream>
#include <vector>
#include <algorithm>

class Control
{
public:
	Control();
	void Callback(const lidar_detection::lidar& lidar_msg);

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;

	geometry_msgs::Twist motor;
};