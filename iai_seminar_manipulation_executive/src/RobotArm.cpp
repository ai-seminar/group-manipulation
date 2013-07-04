#include "../include/iai_seminar_manipulation_executive/RobotArm.h"

RobotArm::RobotArm(ros::NodeHandle& n, const std::string& name)
{
	this->trajectory_client_ = new TrajectoryClient("manipulation_executive_action", true);
}

bool RobotArm::initGoal(const std::vector<std::string>& joint_names, 
		const std::vector<double>& joint_goal_s,
		const std::vector<double>& joint_goal_velocities, 
		double duration) 
{
	for(size_t j = 0; j < joint_names.size(); ++j)
	{
		this->goal_.trajectory.joint_names.push_back(joint_names[j]);
	}

	this->goal_.trajectory.points.resize(1);

	this->goal_.trajectory.points[0].positions.resize(joint_goal_s.size());
	for(size_t j = 0; j < joint_goal_s.size(); ++j)
	{
		this->goal_.trajectory.points[0].positions[j] = joint_goal_s[j];
	}
	
	this->goal_.trajectory.points[0].velocities.resize(joint_goal_velocities.size());
	for(size_t j = 0; j < joint_goal_velocities.size(); ++j)
	{
		this->goal_.trajectory.points[0].velocities[j] = joint_goal_velocities[j];
	}

	this->goal_.trajectory.points[0].time_from_start = ros::Duration(duration);

	return true;
}

bool RobotArm::initGoal(ros::NodeHandle& n)
{
	return true;
}

bool RobotArm::waitForActionServer()
{
	return this->trajectory_client_->waitForServer();
}

bool RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal& goal)
{

	goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
	this->trajectory_client_->sendGoal(goal);
	return true;
}

bool RobotArm::startTrajectory()
{
	return this->startTrajectory(this->goal_);
}
