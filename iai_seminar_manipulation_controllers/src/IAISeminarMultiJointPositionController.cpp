#include <pluginlib/class_list_macros.h>
#include "../../iai_seminar_manipulation_utils/include/iai_seminar_manipulation_utils/ParameterServerUtils.h"
#include "../include/iai_seminar_manipulation_controllers/IAISeminarMultiJointPositionController.h"

void IaiSeminarMultiJointPositionController::command_callback(const std_msgs::Float64MultiArrayConstPtr& msg)
{

}

void IaiSeminarMultiJointPositionController::copy_from_command_buffer()
{

}

bool IaiSeminarMultiJointPositionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{
	this->robot_ = robot;
	
	std::vector<std::string> joints;

	// Load joints
	loadStringVectorFromParameterServer(n, "joints", joints);
	
	for(size_t i=0; i < joints.size(); ++i)
	{
		this->joints_.push_back(robot->getJointState(joints[i]));
	}

	// Initialize realtime_publisher
	this->realtime_publisher_.init(n, "state", this->joints_.size());
	this->realtime_publisher_.msg_.joint_names = joints;

	return true;
}

void IaiSeminarMultiJointPositionController::starting()
{

}


void IaiSeminarMultiJointPositionController::update()
{

}

void IaiSeminarMultiJointPositionController::stopping()
{

}

PLUGINLIB_DECLARE_CLASS(iai_seminar_manipulation_controllers, IaiSeminarMultiJointPositionController, IaiSeminarMultiJointPositionController, pr2_controller_interface::Controller)
