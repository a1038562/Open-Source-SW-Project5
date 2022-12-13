#include "LocalControl.h"

Control::Control()
{
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5); // control Info Publish
	sub = nh.subscribe("/lidar_info", 1, &Control::Callback, this); // Lidar Info Subscribe
}

void Control::Callback(const lidar_detection::lidar& lidar_msg)
{
	if (lidar_msg.angle < 0 && lidar_msg.range <= 0.3 && (lidar_msg.left_dist < 0.5 || lidar_msg.left_dist >= 3.0)) // 좌측 장애물 감지
	{
		motor.angular.z = -0.3; // angle
		motor.linear.x = 0.0; // speed   
		ROS_INFO("Left Obstacle Detected");
		pub.publish(motor);
	}

	else if (lidar_msg.angle >= 0 && lidar_msg.range <= 0.3 && (lidar_msg.right_dist < 0.5 || lidar_msg.right_dist >= 3.0)) // 우측 장애물 감지
	{
		motor.angular.z = 0.3; // angle
		motor.linear.x = 0.0; // speed   
		ROS_INFO("Right Obstacle Detected");
		pub.publish(motor);
	}

	else if (lidar_msg.range > 0.3) // 전진
	{
		motor.angular.z = 0.0; // angle
		motor.linear.x = 0.1; // speed
		ROS_INFO("Go Foward");
		pub.publish(motor);
	}

	// else 
	// {
	//     motor.angular.z = 0.0; // angle
	//     motor.linear.x = 0.0; // speed
	//     ROS_INFO("Stop");
	//     pub.publish(motor); 
	// }

	//ros::Rate loop_rate(5);
	//loop_rate.sleep();
}