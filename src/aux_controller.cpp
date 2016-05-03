/********************************************************************
* Software License Agreement (BSD License)

*  Copyright (c) 2016, Samuel Lindgren
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <train/cmd_aux.h>

// temporary global varaibles
double back_leg_vel_ = 0;
double front_leg_vel_ = 0;

ros::Publisher front_leg_pub;
ros::Publisher back_leg_pub;

void cmd_auxCallback(const train::cmd_aux cmd_aux)
{
	ROS_INFO("Front leg: %f		Back leg: %f", cmd_aux.front_leg_vel, cmd_aux.back_leg_vel);
	front_leg_vel_ = cmd_aux.front_leg_vel;
	back_leg_vel_ = cmd_aux.back_leg_vel;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "aux_controller");
	ros::NodeHandle n;

	back_leg_pub = n.advertise<std_msgs::Float64>("back_leg_vel", 100);
	front_leg_pub = n.advertise<std_msgs::Float64>("front_leg_vel", 100);

	ros::Subscriber cmd_aux_sub = n.subscribe("cmd_aux", 10, cmd_auxCallback);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		std_msgs::Float64 msg;
		msg.data = front_leg_vel_;
		front_leg_pub.publish(msg);
		msg.data = back_leg_vel_;
		back_leg_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

}
