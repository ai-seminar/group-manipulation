#include "../include/iai_seminar_manipulation_executive/RobotArm.h"
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

// Action client for the joint trajectory action 
// used to trigger the arm movement action
TrajClient* traj_client_;

RobotArm::RobotArm(ros::NodeHandle& n, const std::string& name){
	traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
}

bool RobotArm::initGoal(const std::vector<std::string>& joint_names, const std::vector<double>& joint_goals,
            const std::vector<double>& joint_goal_velocities, double duration){

	pr2_controllers_msgs::JointTrajectoryGoal goal;

	// First, the joint names, which apply to all waypoints
	for (size_t j = 0; j < joint_names.size(); ++j)
	{
		goal.trajectory.joint_names.push_back(joint_names[j]);
	}

	// We will have two waypoints in this goal trajectory
	goal.trajectory.points.resize(2);

	// First trajectory point
	// Positions
	int ind = 0;
	goal.trajectory.points[ind].positions.resize(joint_goals.size());
	for (size_t j = 0; j < joint_goal_velocities.size(); ++j)
	{
		goal.trajectory.points[ind].positions[j] = 0.0;
	}

	// Velocities
	goal.trajectory.points[ind].velocities.resize(joint_goal_velocities.size());
	for (size_t j = 0; j < joint_goal_velocities.size(); ++j)
	{
		goal.trajectory.points[ind].velocities[j] = joint_goal_velocities[j];
	}
	// To be reached 1 second after starting along the trajectory
	goal.trajectory.points[ind].time_from_start = ros::Duration(duration);

	// Second trajectory point
	// Positions
	ind += 1;
	goal.trajectory.points[ind].positions.resize(joint_goals.size());
	for (size_t j = 0; j < joint_goal_velocities.size(); ++j)
	{
		goal.trajectory.points[ind].positions[j] = joint_goals[j];
	}

	// Velocities
	goal.trajectory.points[ind].velocities.resize(joint_goal_velocities.size());
	for (size_t j = 0; j < joint_goal_velocities.size(); ++j)
	{
		goal.trajectory.points[ind].velocities[j] = joint_goal_velocities[j];
	}
	// To be reached 2 seconds after starting along the trajectory
	goal.trajectory.points[ind].time_from_start = ros::Duration(duration);

	//we are done; return the goal
	return true;
}

bool RobotArm::initGoal(ros::NodeHandle& n){
//our goal variable
	pr2_controllers_msgs::JointTrajectoryGoal goal;

	// First, the joint names, which apply to all waypoints
	goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
	goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
	goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
	goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
	goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
	goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
	goal.trajectory.joint_names.push_back("l_wrist_roll_joint");

	// We will have two waypoints in this goal trajectory
	goal.trajectory.points.resize(2);

	// First trajectory point
	// Positions
	int ind = 0;
	goal.trajectory.points[ind].positions.resize(7);
	goal.trajectory.points[ind].positions[0] = 0.0;
	goal.trajectory.points[ind].positions[1] = 0.0;
	goal.trajectory.points[ind].positions[2] = 0.0;
	goal.trajectory.points[ind].positions[3] = 0.0;
	goal.trajectory.points[ind].positions[4] = 0.0;
	goal.trajectory.points[ind].positions[5] = 0.0;
	goal.trajectory.points[ind].positions[6] = 0.0;
	// Velocities
	goal.trajectory.points[ind].velocities.resize(7);
	for (size_t j = 0; j < 7; ++j)
	{
		goal.trajectory.points[ind].velocities[j] = 0.0;
	}
	// To be reached 1 second after starting along the trajectory
	goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

	// Second trajectory point
	// Positions
	ind += 1;
	goal.trajectory.points[ind].positions.resize(7);
	goal.trajectory.points[ind].positions[0] = 1.05;
	goal.trajectory.points[ind].positions[1] = 0.1;
	goal.trajectory.points[ind].positions[2] = 0.61;
	goal.trajectory.points[ind].positions[3] = -0.44;
	goal.trajectory.points[ind].positions[4] = -5.6;
	goal.trajectory.points[ind].positions[5] = -0.86;
	goal.trajectory.points[ind].positions[6] = 0.16;
	// Velocities
	goal.trajectory.points[ind].velocities.resize(7);
	for (size_t j = 0; j < 7; ++j)
	{
		goal.trajectory.points[ind].velocities[j] = 0.0;
	}
	// To be reached 2 seconds after starting along the trajectory
	goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

	//we are done; return the goal
	return true;
}

bool RobotArm::waitForActionServer(){
	while(!traj_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the joint_trajectory_action server");
	}
	return true;
}

bool RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal& goal){
	// When to start the trajectory: 1s from now
	goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
	traj_client_->sendGoal(goal);
	return true;
}

bool RobotArm::startTrajectory(){
	return false;
}

int main(int argc, char **argv){
	return 0;
}