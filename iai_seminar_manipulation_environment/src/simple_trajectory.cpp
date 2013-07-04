#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

class RobotArm
{
private:
	TrajClient* traj_client_;

public:
	RobotArm()
	{
		traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
		while(!traj_client_->waitForServer(ros::Duration(5.0))) ROS_INFO("Waiting for the joint_trajectory_action server");
	}

	~RobotArm()
	{
		delete traj_client_;
	}

	void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
	{
		goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
		traj_client_->sendGoal(goal);
	}

	pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
	{
		pr2_controllers_msgs::JointTrajectoryGoal goal;

		goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
		goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
		goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
		goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
		goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
		goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
		goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
	
		goal.trajectory.points.resize(2);

		int ind = 0;
		goal.trajectory.points[ind].positions.resize(7);
		for(size_t j = 0; j < 7; ++j)
		{
			goal.trajectory.points[ind].positions[j] = 0.0;
		}

		goal.trajectory.points[ind].velocities.resize(7);
	
		for(size_t j = 0; j < 7; ++j)
		{
			goal.trajectory.points[ind].velocities[j] = 0.0;
		}

		ind += 1;

		goal.trajectory.points[ind].positions.resize(7);
		goal.trajectory.points[ind].positions[0] = -0.3;
		goal.trajectory.points[ind].positions[1] = 0.2;
		goal.trajectory.points[ind].positions[2] = -0.1;
		goal.trajectory.points[ind].positions[3] = -1.2;
		goal.trajectory.points[ind].positions[4] = 1.5;
		goal.trajectory.points[ind].positions[5] = -0.3;
		goal.trajectory.points[ind].positions[6] = -0.5;

		goal.trajectory.points[ind].velocities.resize(7);

		for(size_t j = 0; j < 7; ++j)
		{
			goal.trajectory.points[ind].velocities[j] = 0.0;
		}

		goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

		return goal;
	}

	actionlib::SimpleClientGoalState getState()
	{
		return traj_client_->getState();
	}
};

	

int main(int argc, char** argv)
{
	ros::init(argc, argv, "manipulation_executive");

	RobotArm arm;

	arm.startTrajectory(arm.armExtensionTrajectory());

	while(!arm.getState().isDone() && ros::ok())
	{
		usleep(50000);
	}

	return 0;
}
