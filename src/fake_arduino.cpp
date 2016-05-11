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
#include <train/Adc.h>
#include <std_msgs/Float64.h>

ros::Publisher adc_pub; 

double SPEED_SCALE = 5;

double front_leg_vel = 0;
double front_leg_pot = 0;
double back_leg_vel = 0;
double back_leg_pot = 0;

void back_leg_velCallback(const std_msgs::Float64 &back_leg_vel_msg)
{
	ROS_INFO("Back leg velocity: %f", back_leg_vel_msg.data);
	back_leg_vel = SPEED_SCALE*back_leg_vel_msg.data;
}

void front_leg_velCallback(const std_msgs::Float64 &front_leg_vel_msg)
{
	ROS_INFO("Front leg velocity: %f", front_leg_vel_msg.data);
	front_leg_vel = SPEED_SCALE*front_leg_vel_msg.data;
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "fake_arduino");
	ros::NodeHandle n;

	adc_pub = n.advertise<train::Adc>("adc", 100);	
	

	ros::Rate loop_rate(10);
	
	ros::Subscriber back_leg_vel_pub = n.subscribe("back_leg_vel", 10, back_leg_velCallback);
	ros::Subscriber front_leg_vel_pub = n.subscribe("front_leg_vel", 10, front_leg_velCallback);


	train::Adc adc_msg;
	while(ros::ok())
	{

		front_leg_pot += front_leg_vel;
		if(front_leg_pot > 1000) front_leg_pot = 0;
		if(front_leg_pot < 0) front_leg_pot = 1000;
		adc_msg.adc0 = front_leg_pot;	


		back_leg_pot += back_leg_vel;
		if(back_leg_pot > 1000) back_leg_pot = 0;
		if(back_leg_pot < 0) back_leg_pot = 1000;
		adc_msg.adc1 = back_leg_pot;

		adc_pub.publish(adc_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}
