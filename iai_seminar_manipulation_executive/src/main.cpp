#include "ros/ros.h"

int main(int argc, char **argv)
{
	// Ros-Init with Node manipulation_executive
	ros::init(argc, argv, "manipulation_executive");

	// Node-Handle nh
	ros::NodeHandle nh;

	// Spin
	ros::spin();

	return 0;
}