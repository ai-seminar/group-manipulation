#include <../include/iai_seminar_manipulation_executive/RobotArm.h>


using namespace std;
using namespace ros;


RobotArm::RobotArm(ros::NodeHandle& n, const std::string& name){
	trajectory_client_ = new TrajectoryClient(n.getNamespace(), true);
}

bool RobotArm::initGoal(const std::vector<std::string>& joint_names, const std::vector<double>& joint_goals,
                const std::vector<double>& joint_goal_velocities, double duration){
    // First, the joint names, which apply to all waypoints
	for (std::vector<std::string>::const_iterator it = joint_names.begin() ; it != joint_names.end(); ++it){
		goal_.trajectory.joint_names.push_back(*it);
	}
    
    // We will have two waypoints in this goal trajectory
    goal_.trajectory.points.resize(2);
    int ind = 0;
    goal_.trajectory.points[ind].positions.resize(joint_goals.size());
 	for (size_t i = 0 ; i<joint_goals.size(); ++i){
		goal_.trajectory.points[ind].positions[i] = 0.0;
	}  
	
    // Velocities
    goal_.trajectory.points[ind].velocities.resize(joint_goal_velocities.size());
    for (size_t j = 0; j < joint_goals.size(); ++j)
    {
      goal_.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal_.trajectory.points[ind].time_from_start = ros::Duration(1.0);
    
    
    // Second trajectory point
    // Positions
    ind += 1;
    int i = 0;
    goal_.trajectory.points[ind].positions.resize(joint_goals.size());
    
    for (std::vector<double>::const_iterator it = joint_goals.begin() ; it != joint_goals.end(); ++it, ++i){
		goal_.trajectory.points[ind].positions[i] = *it;
	}

    // Velocities
    goal_.trajectory.points[ind].velocities.resize(joint_goal_velocities.size());
    i = 0;
    for (std::vector<double>::const_iterator it = joint_goal_velocities.begin() ; it != joint_goal_velocities.end(); ++it, ++i){
		goal_.trajectory.points[ind].velocities[i] = *it;
	}    

    // To be reached 2 seconds after starting along the trajectory
    goal_.trajectory.points[ind].time_from_start = ros::Duration(2.0);

	goal_.trajectory.header.stamp = ros::Time::now() + ros::Duration(duration);
    //we are done; return the goal
    //return goal;
	return true;
}

bool RobotArm::initGoal(ros::NodeHandle& n){
	return false;
}	

bool RobotArm::waitForActionServer(){
	while(!trajectory_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the joint_trajectory_action server");
	}
	return true;
}

bool RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal& goal){
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    trajectory_client_->sendGoal(goal);
	return true;
}

bool RobotArm::startTrajectory(){

	trajectory_client_->sendGoal(goal_);
	return true;
}












