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
geometry_msgs::Twist motor;

 Control()
    {
        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
        sub = nh.subscribe("/lidar_detect", 1,  &Control::Callback, this);
    }

    void Callback(const lidar_detection::lidar &msg)
    {       
        ros::Rate loop_rate(5);

        if (msg.range < 0.5)
        {
            motor.linear.x = 0;
        }
   
        pub.publish(motor);
        
        loop_rate.sleep();
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_control");
    Control controler;
    ros::spin();
}