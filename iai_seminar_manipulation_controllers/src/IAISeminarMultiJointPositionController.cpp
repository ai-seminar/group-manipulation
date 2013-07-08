#include <pluginlib/class_list_macros.h>
#include "../../iai_seminar_manipulation_utils/include/iai_seminar_manipulation_utils/ParameterServerUtils.h"
#include "../include/iai_seminar_manipulation_controllers/IAISeminarMultiJointPositionController.h"

using namespace std;

ros::NodeHandle nh;



void IaiSeminarMultiJointPositionController::command_callback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
	cout << "dsdasd \n" ;
	if(command_mutex_.try_lock())
	{
		cout << "hier callback?\n";
		position_command_buffer_ = msg->data;
		cout << "hier cdsadsallback?\n";
		command_mutex_.unlock();
		cout << "hier callbadsadasck?\n";
	}
}

void IaiSeminarMultiJointPositionController::copy_from_command_buffer()
{
	position_command_ = position_command_buffer_;
	position_command_buffer_.clear();
}

bool IaiSeminarMultiJointPositionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{
	//nh = n;
	robot_ = robot;
	std::vector<std::string> joints;
	
	// Load joints
	loadStringVectorFromParameterServer(n, "/joints", joints);
	cout << joints.size() << endl;
	for(size_t i=0; i < joints.size(); ++i)
	{
		pr2_mechanism_model::JointState* js = robot->getJointState(joints[i]);
		if(!js)
		{
			// Error while initialising joint "joints[i]"
			ROS_ERROR("IaiSeminarMultiJointPositionController could not find joint '%s'", joints[i].c_str());
			return false;
		}
		joints_.push_back(js);
		
		
		control_toolbox::Pid pid;
		if (!pid.initParam(n.getNamespace()+"/gains/"+joints[i])){
			ROS_ERROR("IaiSeminarMultiJointPositionController could not construct PID controller for joint", joints[i].c_str());
			return false;		
		}
		pids_.push_back(pid);
		/*double p = 0;
		double i = 0;
		double d = 0;
		double imax = 0;
		double imin = 0;
		pids_[i].getGains(p,i,d,imax,imin);
		cout << "p: " << p << endl;*/
	}

	// Initialize realtime_publisher
	realtime_publisher_.init(n, "state", joints_.size());
	realtime_publisher_.msg_.joint_names = joints;

	return true;
}

void IaiSeminarMultiJointPositionController::starting()
{
	for(size_t i=0; i < pids_.size(); ++i){
		pids_[i].reset();
		error_.push_back(0);
	}
	last_time_ = robot_->getTime();
}


void IaiSeminarMultiJointPositionController::update()
{
	cout << "spam" << endl;
	if(command_mutex_.try_lock())
	{
		copy_from_command_buffer();
		command_mutex_.unlock();
	}
	//IaiSeminarMultiJointPositionController bla;
	cout << "dasda\n" ;
	//command_subscriber_ = nh.subscribe("/command", 1, &IaiSeminarMultiJointPositionController::command_callback, &bla);
	cout << ":(\n" ;
	if(realtime_publisher_.trylock())
	{
		// Update current position of joints
		for(size_t i=0; i < joints_.size(); ++i)
		{
			cout << "hier?\n";
			realtime_publisher_.msg_.actual.positions.push_back(joints_[i]->position_);
			realtime_publisher_.msg_.actual.velocities.push_back(joints_[i]->velocity_);
			cout << "hier2?\n";
			double desired_pos = 0//position_command_[i];
			double current_pos = 0//joints_[i]->position_;
			cout << "hier3?\n";
			ros::Duration dt = robot_->getTime() - last_time_;
			last_time_ = robot_->getTime();
			cout << "hier4?\n";
			pids_[i].updatePid(error_[i], dt);
			error_[i] =current_pos-desired_pos;
			cout << "nein\n";
			
		}
		// Publish
		realtime_publisher_.unlockAndPublish();
	}
}

void IaiSeminarMultiJointPositionController::stopping()
{

}

PLUGINLIB_DECLARE_CLASS(iai_seminar_manipulation_controllers, IaiSeminarMultiJointPositionController, IaiSeminarMultiJointPositionController, pr2_controller_interface::Controller)
