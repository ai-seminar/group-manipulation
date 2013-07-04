#include <ros/ros.h>
#include "../include/iai_seminar_manipulation_executive/RobotArm.h"

int main(int argc, char** argv)
{

	std::vector<std::string> joint_names;
	joint_names.push_back("l_shoulder_pan_joint");
	joint_names.push_back("l_shoulder_lift_joint");
	joint_names.push_back("l_upper_arm_roll_joint");
	joint_names.push_back("l_elbow_flex_joint");
	joint_names.push_back("l_forearm_roll_joint");
	joint_names.push_back("l_wrist_flex_joint");
	joint_names.push_back("l_wrist_roll_joint");

	std::vector<double> positions;
	positions.push_back(1.05);
	positions.push_back(0.1);
	positions.push_back(0.61);
	positions.push_back(-0.44);
	positions.push_back(-5.6);
	positions.push_back(-0.89);
	positions.push_back(0.16);

	std::vector<double> velocities;
	velocities.push_back(0.0);
	velocities.push_back(0.0);
	velocities.push_back(0.0);
	velocities.push_back(0.0);
	velocities.push_back(0.0);
	velocities.push_back(0.0);
	velocities.push_back(0.0);

	double duration = 3.0;
	
	ros::init(argc, argv, "manipulation_executive");

	ros::NodeHandle n;

	std::string name = "manipulation_executive";
	
	RobotArm arm = RobotArm(n, name);
	arm.waitForActionServer();
	arm.initGoal(joint_names, positions, velocities, duration);
	arm.startTrajectory();
	
	return 0;	
}
