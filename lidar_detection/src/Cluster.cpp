#include "Cluster.h"

CloudCluster::CloudCluster()
{
	pub_info = nh.advertise<lidar_detection::lidar>("/lidar_info", 1); // Lidar Info Publish
	pub = nh.advertise<sensor_msgs::PointCloud2>("/cluster", 1); // cluster result Publish
	sub = nh.subscribe("/pcl2", 1, &CloudCluster::Callback, this); // PCL2 Subscribe
}

void CloudCluster::Callback(const sensor_msgs::PointCloud2ConstPtr& pcl2_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::fromROSMsg(*pcl2_msg, *cloud_ptr); // ROS Message cloud2를 cloud로 변환  
	tree->setInputCloud(cloud_ptr);

	// Euclidean Cluster Extraction
	ec.setClusterTolerance(0.4);    // Point 사이 최소 허용 거리
	ec.setMinClusterSize(5);    // Min Cluster Size
	ec.setMaxClusterSize(50);   // Max Cluster Size
	ec.setSearchMethod(tree);   // Search Method - tree 
	ec.setInputCloud(cloud_ptr);    // Clustering result
	ec.extract(cluster_indices);

	i = 0;
	total_cloud.clear();

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			pt = cloud_ptr->points[*pit];
			if (pt.z == 0)
			{
				pt2.x = pt.x;
				pt2.y = pt.y;
				pt2.z = pt.z;
				pt2.intensity = (float)(i);
				total_cloud.push_back(pt2);

				pt_dist = std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2));
				pt_angle = atan2f(pt.x, pt.y) * 180 / M_PI;

				if (pt_dist > 0.2 && pt_dist < min_dist) // min distance 
				{
					min_dist = pt_dist;
					min_point.x = pt.x;
					min_point.y = pt.y;
				}

				if (pt_dist > 0.2 && pt_dist < left_min && pt_angle < 0)
				{
					left_min = pt_dist;
				}
				else if (pt_dist > 0.2 && pt_dist < left_min && pt_angle >= 0)
				{
					right_min = pt_dist;
				}
			}
		}
		i++;
	}

	// -70 < dist_angle < 0: 좌
	// 0 < dist_angle < 70: 우 

	final_dist = std::sqrt(std::pow(min_point.x, 2) + std::pow(min_point.y, 2));
	dist_angle = atan2f(min_point.x, min_point.y) * 180 / M_PI;

	ROS_INFO("min distance: %.4f", final_dist);

	lidar_info.angle = dist_angle;
	lidar_info.range = final_dist;
	lidar_info.left_dist = left_min;
	lidar_info.right_dist = right_min;
	pub_info.publish(lidar_info);

	min_dist = 1.0;
	left_min = 2.0;
	right_min = 2.0;

	pcl::toPCLPointCloud2(total_cloud, clustered); // cloud를 cloud2로 변환
	pcl_conversions::fromPCL(clustered, output);
	output.header.frame_id = "map";
	pub.publish(output);

	//ros::Rate loop_rate(5);
	//loop_rate.sleep(); 
}

