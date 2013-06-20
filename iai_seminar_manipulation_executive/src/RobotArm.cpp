#include "../include/iai_seminar_manipulation_executive/RobotArm.h"

RobotArm::RobotArm(ros::NodeHandle& n, const std::string& name){
}

bool RobotArm::initGoal(const std::vector<std::string>& joint_names, const std::vector<double>& joint_goals,
            const std::vector<double>& joint_goal_velocities, double duration){
	return false;
}

bool RobotArm::initGoal(ros::NodeHandle& n){
	return false;
}

bool RobotArm::waitForActionServer(){
	return false;
}

bool RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal& goal){
	return false;
}

bool RobotArm::startTrajectory(){
	return false;
}

int main(int argc, char **argv){
	return 0;
}