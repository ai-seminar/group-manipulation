#include <../include/iai_seminar_manipulation_executive/RobotArm.h>
#include <../../iai_seminar_manipulation_utils/include/iai_seminar_manipulation_utils/ParameterServerUtils.h>

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
    
    goal_.trajectory.points.resize(1);
    int ind = 0;

    // Positions
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
    goal_.trajectory.points[ind].time_from_start = ros::Duration(1.0);

	goal_.trajectory.header.stamp = ros::Time::now() + ros::Duration(duration);
    
	return true;
}

bool RobotArm::initGoal(ros::NodeHandle& n){
	//get joints
	std::vector<std::string>joint;
	loadStringVectorFromParameterServer(n,"/first_goal_configuration/joints",joint);

	//get positions
	std::vector<double>positions;
	loadDoubleVectorFromParameterServer(n,"/first_goal_configuration/positions",positions);
	
	//get velocities
	std::vector<double>velocities;
	loadDoubleVectorFromParameterServer(n,"/first_goal_configuration/velocities",velocities);
	
	//get executiontime
	double execution_time;
	loadDoubleFromParameterServer(n,"/first_goal_configuration/execution_time",execution_time);	
	
	return initGoal(joints, positions, velocities, executions_time);
}	

bool RobotArm::waitForActionServer(){
	while(!trajectory_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the joint_trajectory_action server");
	}
	return true;
}

bool RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal& goal){
	//Wozu brauch man das?!?
	return trajectory_client_->sendGoal(goal);
}

bool RobotArm::startTrajectory(){
	return trajectory_client_->sendGoal(goal_);
}
