#include "Scan2PCL.h"
#include "lidar_detection/lidar.h" 
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>

class CloudCluster
{
public:
	CloudCluster();
	void Callback(const sensor_msgs::PointCloud2ConstPtr& pcl2_msg);

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Publisher pub_info;
	ros::Subscriber sub;

	sensor_msgs::PointCloud2 output;
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	pcl::PointCloud<pcl::PointXYZI> total_cloud;
	pcl::PCLPointCloud2 clustered;
	pcl::PointXYZ pt;
	pcl::PointXYZI pt2;
	int i = 0;
	bool init = false;
	float min_dist = 1.0;
	float left_min = 2.0;
	float right_min = 2.0;
	cv::Point2f min_point;
	lidar_detection::lidar lidar_info;
	float pt_dist, pt_angle, final_dist, dist_angle;
};