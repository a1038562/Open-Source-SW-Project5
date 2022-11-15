#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "lidar_detection/lidar.h"
#include <iostream>
#include <vector>
#include <algorithm>

// 센서 각도 +15도

class LidarDetect
{
public:
	sensor_msgs::LaserScan laser_msg;
	LidarDetect()
	{
		pub = nh.advertise<lidar_detection::lidar>("/lidar_detect", 1);
		sub = nh.subscribe("/scan", 1, &LidarDetect::Callback, this);
	}

	void Callback(const sensor_msgs::LaserScan::ConstPtr &msg)
	{
		laser_msg = *msg;

		std::vector<float> laser_ranges;
		int index;
		float min_range = laser_msg.range_max;
		float angle;
		lidar_detection::lidar lidar_topic;
		ros::Rate loop_rate(30);

		laser_ranges = laser_msg.ranges;
		size_t range_size = laser_ranges.size();

		for (size_t i = 0; i < range_size; i++)
			if (laser_ranges[i] < min_range && laser_ranges[i] != 0) {
				min_range = laser_ranges[i];
				index = i;
			}

		angle = laser_msg.angle_min + 360 * (float(index) / float(range_size));

		lidar_topic.range = min_range;
		lidar_topic.angle = angle;
		pub.publish(lidar_topic);

		loop_rate.sleep();
	}

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_detect");
	LidarDetect lidar;
	ros::spin();
}
