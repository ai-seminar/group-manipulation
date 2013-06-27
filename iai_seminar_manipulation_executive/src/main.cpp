#include <ros/ros.h>
#include <../include/iai_seminar_manipulation_executive/RobotArm.h>
#include <../../iai_seminar_manipulation_utils/include/iai_seminar_manipulation_utils/ParameterServerUtils.h>


using namespace ros;
using namespace std;


std::vector<std::string> bla1(){
	vector<string> muh;
	muh.push_back("l_shoulder_pan_joint");
    muh.push_back("l_shoulder_lift_joint");
    muh.push_back("l_upper_arm_roll_joint");
    muh.push_back("l_elbow_flex_joint");
    muh.push_back("l_forearm_roll_joint");
    muh.push_back("l_wrist_flex_joint");
    muh.push_back("l_wrist_roll_joint");
    return muh;
}

std::vector<double> bla2(){
	std::vector<double> muh;
	muh.push_back(1.05);
	muh.push_back(0.1);
	muh.push_back(0.61);
	muh.push_back(-0.44);
	muh.push_back(-5.6);
	muh.push_back(-0.86);
	muh.push_back(0.16);
	return muh;
}

std::vector<double> bla3(){
	std::vector<double> muh;
	muh.push_back(0.0);
	muh.push_back(0.0);
	muh.push_back(0.0);
	muh.push_back(0.0);
	muh.push_back(0.0);
	muh.push_back(0.0);
	muh.push_back(0.0);	
	return muh;
}

int main(int argc, char **argv){
	init(argc, argv, "manipulation_executive");
	
	NodeHandle n;
	cout << n.getNamespace()<< endl;
	std::vector<std::string>joint;
	loadStringVectorFromParameterServer(n,"/first_goal_configuration/joints",joint);
	cout << *joint.begin() << endl;
	std::vector<double>positions;
	loadDoubleVectorFromParameterServer(n,"/first_goal_configuration/positions",positions);
	std::vector<double>velocities;
	loadDoubleVectorFromParameterServer(n,"/first_goal_configuration/velocities",velocities);
	double execution_time;
	loadDoubleFromParameterServer(n,"/first_goal_configuration/execution_time",execution_time);		
	//loadStringVectorFromParameterServer(n,"/first_goal_configuration/execution_time",muh);
	
	RobotArm armi(n,"muh");
	armi.waitForActionServer();
	armi.initGoal(joint,positions,velocities,execution_time);
	armi.startTrajectory();
	// Wait for trajectory completion
	while(!armi.getState().isDone() && ros::ok())
	{
		usleep(10000);
	}
	//spin();
}










/*
bool RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal&)
bool RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal)
*/








