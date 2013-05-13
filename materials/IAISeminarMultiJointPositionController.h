/*  Copyright (c) 2013, Georg Bartels (georg.bartels@cs.uni-bremen.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *   
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the Institute for Artificial Intelligence/Universität Bremen
 *      nor the names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IAISEMINARMULTIJOINTPOSITIONCONTROLLER_H_
#define IAISEMINARMULTIJOINTPOSITIONCONTROLLER_H_

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>

#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <realtime_tools/realtime_publisher.h>

#include <std_msgs/Float64MultiArray.h>

#include <boost/thread/recursive_mutex.hpp>

#include <control_toolbox/pid.h>

#include <vector>

class IaiSeminarMultiJointPositionController: public pr2_controller_interface::Controller
{
private:
  // internal pointers to all the joints to control
  std::vector<pr2_mechanism_model::JointState*> joints_;
  // internal pointer to the robot object
  pr2_mechanism_model::RobotState* robot_;
  // vector holding a pid-controller for every joint to control
  std::vector<control_toolbox::Pid> pids_; 

  // container to store the last time our update was called, needed to calculate time between cycles
  ros::Time last_time_;
  // container to hold the calculated error between actual and desired positions of all joints to control
  std::vector<double> error_;
 
  // a real-time-safe publisher to publish the state of our controller
  realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointTrajectoryControllerState> realtime_publisher_; 

  // double buffers to read command in a realtime-safe fashion
  std::vector<double> position_command_, position_command_buffer_;
  // mutex to guard operations on command buffer
  boost::mutex command_mutex_;

  // subscriber object used to listen to command topic
  ros::Subscriber command_subscriber_;
  // callback to copy in desired joint positions from topic
  void command_callback(const std_msgs::Float64MultiArrayConstPtr& msg);

  // auxiliary function that copies content of position_command_buffer_ into position_command_
  // uses mutex to guard against interference and appear atomic
  void copy_from_command_buffer();
public:
  // standard interface of pr2 controllers...

  // init gets called once when loading the controller; no need for realtime safety
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n);

  // real-time safe: called once when starting the controller to set it up
  virtual void starting();
 
  // real-time safe: called once every 1ms to computer control signal
  virtual void update();

  // real-time safe: called once when stopping the controller
  virtual void stopping();
};

#endif /* IAISEMINARMULTIJOINTPOSITIONCONTROLLER_H_ */
