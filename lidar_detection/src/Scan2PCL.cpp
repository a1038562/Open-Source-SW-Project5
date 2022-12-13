#include "Scan2PCL.h"

Scan2PCL::Scan2PCL()
{
	pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl2", 1); //PCL2 Publish
	sub = nh.subscribe("/scan", 1, &Scan2PCL::Callback, this); //LaserScan Subscribe
}

void Scan2PCL::Callback(const sensor_msgs::LaserScan laser_msg)
{
	cloud.clear();

	laser_ranges = laser_msg.ranges;
	range_size = laser_ranges.size();

	// ROI
	for (size_t i = 0; i < range_size; i++)
	{
		if (laser_ranges[i] > min_range && laser_ranges[i] < max_range) { // ROI range
			angle = laser_msg.angle_min + 360.0 * (float(i) / float(range_size));
			if ((290.0 <= angle && angle <= 360.0) || (0.0 <= angle && angle <= 70.0)) // ROI angle
			{
				x = (-1) * std::get<0>(coordinate);
				y = (1) * std::get<1>(coordinate);
				coordinate = coordinate_calc(laser_ranges[i], angle);
				cloud.push_back(pcl::PointXYZ(x, y, 0));
			}
		}
	}

	cloud2 = toCloud2(cloud);
	pub.publish(cloud2);

	//ros::Rate loop_rate(5);
	//loop_rate.sleep();
}

// Point Cloud coordinate calculation
std::tuple<float, float> Scan2PCL::coordinate_calc(float range, float angle)
{
	float x, y;
	x = range * sin(angle * M_PI / 180);
	y = range * cos(angle * M_PI / 180);

	return std::make_tuple(x, y);
}

// PointCloud to PointCloud2 Sensor Messages
sensor_msgs::PointCloud2 Scan2PCL::toCloud2(pcl::PointCloud<pcl::PointXYZ> cloud)
{
	sensor_msgs::PointCloud2 cloud2;
	pcl::toROSMsg(cloud, cloud2);
	cloud2.header.frame_id = "map";
	return cloud2;
}