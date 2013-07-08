#include <ros/ros.h>
#include <../include/iai_seminar_manipulation_executive/RobotArm.h>
#include <../../iai_seminar_manipulation_utils/include/iai_seminar_manipulation_utils/ParameterServerUtils.h>


using namespace ros;
using namespace std;


int main(int argc, char **argv){
	init(argc, argv, "manipulation_executive");
	
	NodeHandle n;

	RobotArm armi(n,"muh");
	armi.waitForActionServer();
	armi.initGoal();
	armi.startTrajectory();
	// Wait for trajectory completion
	while(!armi.getState().isDone() && ros::ok())
	{
		usleep(50000);
	}
	spin();
}














