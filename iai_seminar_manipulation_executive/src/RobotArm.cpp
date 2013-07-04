// RobotArm.cpp

#include "../include/iai_seminar_manipulation_executive/RobotArm.h"

	typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

	TrajClient* traj_client_;

	// traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);


	bool RobotArm::initGoal(const std::vector<std::string>& joint_names, const std::vector<double>& joint_goals,
            const std::vector<double>& joint_goal_velocities, double duration){

		// Setting up new Goal
		pr2_controllers_msgs::JointTrajectoryGoal goal;

		// Defining the joint's names
		for(unsigned i=0; i < joint_names.size(); i++) {
    		goal.trajectory.joint_names.push_back(joint_names[i]);
		}

		// We have only one goal
		goal.trajectory.points.resize(1);

		// 
		goal.trajectory.points[0].positions.resize(joint_goals.size());

		// Defining the positions
		for(unsigned i=0; i < joint_goals.size(); i++) {
    		goal.trajectory.points[0].positions[i] = joint_goals[i];
		}

		// Velocities
       	goal.trajectory.points[0].velocities.resize(7);
       	for (size_t j = 0; j < 7; ++j) {
         	goal.trajectory.points[0].velocities[j] = joint_goal_velocities[j];
      	}

      	// Defining the duration
      	goal.trajectory.points[0].time_from_start = ros::Duration(duration);


		return true;
	}

	bool RobotArm::initGoal(ros::NodeHandle& n){
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
	}

	bool RobotArm::startTrajectory(){
		return true;
	}
