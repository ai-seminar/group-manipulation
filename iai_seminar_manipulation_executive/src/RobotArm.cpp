#include "../include/iai_seminar_manipulation_executive/RobotArm.h"

RobotArm::RobotArm(ros::NodeHandle& n, const std::string& name)
{
}

bool RobotArm::initGoal(const std::vector<std::string>& joint_names, 
		const std::vector<double>& joint_goals,
		const std::vector<double>& joint_goal_velocities, 
		double duration) 
{
	return true;
}

bool RobotArm::initGoal(ros::NodeHandle& n)
{
	return true;
}

bool RobotArm::waitForActionServer()
{
	return true;
}

bool startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal& goal)
{
	return true;
}

bool startTrajectory()
{
	return true;
}
