#include "ros/ros.h"
#include "../include/iai_seminar_manipulation_executive/RobotArm.h"

int main(int argc, char **argv)
{

	
	// Ros-Init with Node manipulation_executive
	ros::init(argc, argv, "manipulation_executive");

	// Node-Handle nh
	ros::NodeHandle nh;
	
	RobotArm(nh, "arm_action") arm_action;
	arm_action.initGoal(nh);
	// Spin
	ros::spin();

	return 0;
}