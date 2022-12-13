#include "LocalControl.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Local_Control");
	Control control;
	ros::spin();
}