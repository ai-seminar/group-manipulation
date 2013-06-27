#include <ros/ros.h>
#include "../include/iai_seminar_manipulation_executive/RobotArm.h"

int main(int argc, char** argv)
{

	ros::init(argc, argv, "manipulation_executive");

	ros::NodeHandle n;

	ros::spin();
	
	return 0;	
}
