// RobotArm.cpp

#include "../include/iai_seminar_manipulation_executive/RobotArm.h"


	bool RobotArm::initGoal(const std::vector<std::string>& joint_names, const std::vector<double>& joint_goals,
            const std::vector<double>& joint_goal_velocities, double duration){
		return true;
	}

	bool RobotArm::initGoal(ros::NodeHandle& n){
		return true;
	}

	bool RobotArm::waitForActionServer(){

	}

	bool RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal& goal){
		return true;
	}

	bool RobotArm::startTrajectory(){
		return true;
	}