#include <pluginlib/class_list_macros.h>
#include <../../iai_seminar_manipulation_utils/include/iai_seminar_manipulation_utils/ParameterServerUtils.h>
#include <../include/iai_seminar_manipulation_controllers/IAISeminarMultiJointPositionController.h>

using namespace std;

void IaiSeminarMultiJointPositionController::command_callback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
	//empfangende Daten zwischenspeichern
	boost::mutex::scoped_lock lock(command_mutex_);
	position_command_buffer_ = msg->data;

}

void IaiSeminarMultiJointPositionController::copy_from_command_buffer()
{
	//Daten aus dem Buffer laden
	boost::mutex::scoped_lock lock(command_mutex_);
	position_command_ = position_command_buffer_;
}

bool IaiSeminarMultiJointPositionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{

	robot_ = robot;
	std::vector<std::string> joints;
	
	// lade joints
	loadStringVectorFromParameterServer(n, "/joints", joints);
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
		
		//PIDs holen und speichern
		control_toolbox::Pid pid;
		if (!pid.initParam(n.getNamespace()+"/gains/"+joints[i])){
			ROS_ERROR("IaiSeminarMultiJointPositionController could not construct PID controller for joint", joints[i].c_str());
			return false;		
		}
		pids_.push_back(pid);
		
	}

	//initialisiere realtime_publisher
	realtime_publisher_.init(n, "state", 4);
	realtime_publisher_.msg_.joint_names = joints;
	realtime_publisher_.msg_.actual.positions.resize(joints.size());
	realtime_publisher_.msg_.actual.velocities.resize(joints.size());
	realtime_publisher_.msg_.desired.positions.resize(joints.size());
	realtime_publisher_.msg_.error.positions.resize(joints.size());
	
	//starte command-listener
	command_subscriber_ = n.subscribe(n.getNamespace()+"/command", 1, &IaiSeminarMultiJointPositionController::command_callback, &*this);
	return true;
}

void IaiSeminarMultiJointPositionController::starting()
{	
	position_command_buffer_.resize(pids_.size());
	position_command_.resize(pids_.size());
	
	for(size_t i=0; i < pids_.size(); ++i){
		//reset pids 
		pids_[i].reset();
		
		//error was zuweisen.. weil sonst irgendwie alles kaputt geht
		error_.push_back(0);
		
		//sinnvolle Startposition
		position_command_buffer_[i]=0;
		position_command_[i]=0;
	}
	cout << "Controller started" << endl;
	//Startzeit merken
	last_time_ = robot_->getTime();
}


void IaiSeminarMultiJointPositionController::update()
{
	copy_from_command_buffer();
	
	if(realtime_publisher_.trylock())
	{
		// Update current position of joints	
		
		
		for(size_t i=0; i < joints_.size(); ++i)
		{
			double desired_pos = position_command_[i];
			double current_pos = joints_[i]->position_;
			
			ros::Duration dt = robot_->getTime() - last_time_;
			last_time_ = robot_->getTime();
			
			pids_[i].updatePid(error_[i], dt);
			error_[i] =desired_pos-current_pos;
			
			joints_[i]->commanded_effort_ =  error_[i];
			
			//publish Status			
			realtime_publisher_.msg_.actual.positions[i] = joints_[i]->position_;
			realtime_publisher_.msg_.actual.velocities[i] = joints_[i]->velocity_;
			realtime_publisher_.msg_.desired.positions[i] = position_command_[i];	
			realtime_publisher_.msg_.error.positions[i] = error_[i];
		}
		// Publish
		realtime_publisher_.unlockAndPublish();
	}

}

void IaiSeminarMultiJointPositionController::stopping()
{

}

PLUGINLIB_DECLARE_CLASS(iai_seminar_manipulation_controllers, IaiSeminarMultiJointPositionController, IaiSeminarMultiJointPositionController, pr2_controller_interface::Controller)
