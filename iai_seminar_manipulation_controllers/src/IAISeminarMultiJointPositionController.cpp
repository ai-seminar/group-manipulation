#include <pluginlib/class_list_macros.h>
#include "../../iai_seminar_manipulation_utils/include/iai_seminar_manipulation_utils/ParameterServerUtils.h"
#include "../include/iai_seminar_manipulation_controllers/IAISeminarMultiJointPositionController.h"

void IaiSeminarMultiJointPositionController::command_callback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
	if(this->command_mutex_.try_lock())
	{
		this->position_command_buffer_ = msg->data;
		this->command_mutex_.unlock();
	}
}

void IaiSeminarMultiJointPositionController::copy_from_command_buffer()
{
	this->position_command_ = this->position_command_buffer_;
	this->position_command_buffer_.clear();
}

bool IaiSeminarMultiJointPositionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{
	this->robot_ = robot;
	
	std::vector<std::string> joints;

	// Load joints
	loadStringVectorFromParameterServer(n, "joints", joints);
	
	for(size_t i=0; i < joints.size(); ++i)
	{
		pr2_mechanism_model::JointState* js = robot->getJointState(joints[i]);
		if(!js)
		{
			// Error while initialising joint "joints[i]"
			ROS_ERROR("IaiSeminarMultiJointPositionController could not find joint '%s'", joints[i].c_str());
			return false;
		}
		this->joints_.push_back(js);
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
	if(this->command_mutex_.try_lock())
	{
		this->copy_from_command_buffer();
		this->command_mutex_.unlock();
	}

	if(this->realtime_publisher_.trylock())
	{
		// Update current position of joints
		for(size_t i=0; i < this->joints_.size(); ++i)
		{
			this->realtime_publisher_.msg_.actual.positions.push_back(this->joints_[i]->position_);
			this->realtime_publisher_.msg_.actual.velocities.push_back(this->joints_[i]->velocity_);
		}
		// Publish
		this->realtime_publisher_.unlockAndPublish();
	}
}

void IaiSeminarMultiJointPositionController::stopping()
{

}

PLUGINLIB_DECLARE_CLASS(iai_seminar_manipulation_controllers, IaiSeminarMultiJointPositionController, IaiSeminarMultiJointPositionController, pr2_controller_interface::Controller)
