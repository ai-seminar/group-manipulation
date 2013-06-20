// First ROS node

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
using namespace std;

int main (int argc, char **argv)
{
  cout << "Hello World!";

  ros::init(argc,argv,"manipulation_executive");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::String>("manipulation_executive_chatter",1000);

  ros::Rate loop_rate(10);

  while(ros::ok()){
  	std_msgs::String msg;
  	msg.data = "hello world";
  	pub.publish(msg);
  	ros::spinOnce();
  	loop_rate.sleep();
  }


  return 0;
}