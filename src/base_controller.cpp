/*********************************************************************
* Software License Agreement (BSD License)
*
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
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

//#include "std_msgs/String.h"
//#include <sstream>
//#include <iostream>

#include <std_msgs/UInt16.h>
//#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <train/base_controllerConfig.h>


ros::Publisher servo_pub;
ros::Publisher servo_angle_pub;

// TODO: change from using static pos_x and pos_y to using tf
class Wheel
{
public:
	Wheel(int initial_angle, int initial_speed, double pos_x, double pos_y);
	void move(const geometry_msgs::Twist &twist_aux);
	int get_speed();
	int get_angle();
	int get_center_speed_bias();
	double get_speed_scale();


	void set_speed(int speed);
	void set_angle(int angle);
	void set_center_speed_bias(int center_speed_bias);
	void set_speed_scale(double speed_scale);

private:
	int speed_, angle_;
	double pos_x_, pos_y_;
	int center_speed_bias_;
	double speed_scale_;
};

Wheel::Wheel(int initial_angle, int initial_speed, double pos_x, double pos_y)
{
	angle_ = initial_angle;
	speed_ = initial_speed;
	pos_x_ = pos_x;
	pos_y_ = pos_y;

	// dynamic reconfigure should overwrite theese
	center_speed_bias_ = 0; 
	speed_scale_ = 1;
}

int Wheel::get_angle()
{
	// TODO: add mutex ?
	return angle_;
}

int Wheel::get_speed()
{
	// TODO: add mutex ?
	return speed_;
}

int Wheel::get_center_speed_bias()
{
	return center_speed_bias_;
}

double Wheel::get_speed_scale()
{
	return speed_scale_;
}


void Wheel::set_angle(int angle)
{
	// TODO: add mutex ?
	angle_ = angle;
}

void Wheel::set_speed(int speed)
{
	// TODO: add mutex ?
	speed_ = speed;
}

void Wheel::set_center_speed_bias(int center_speed_bias)
{
	center_speed_bias_ = center_speed_bias;
}

void Wheel::set_speed_scale(double speed_scale)
{
	speed_scale_ = speed_scale;
}


void Wheel::move(const geometry_msgs::Twist &twist_aux)
{
	// calculate transveral velocity and angle
	double vel_x = twist_aux.linear.x;
	double vel_y = twist_aux.linear.y;

	// TODO: check if this works as expected on robot...
	double phi = atan2(pos_y_, pos_x_);
	double r = sqrt(pos_x_ * pos_x_ + pos_y_ * pos_y_);

	// ROS_INFO("Twist angular z %f", twist_aux.angular.z);

	if(fabs(twist_aux.angular.z) > 0.05)
	{
		// TODO: scaling?
		vel_x += twist_aux.angular.z*r*cos(phi+3.14/2);
		vel_y += twist_aux.angular.z*r*sin(phi+3.14/2);
		// ROS_INFO("vel_x = %f	vel_y = %f", vel_x, vel_y);
	}

	double trans_vel = sqrt(vel_x * vel_x + vel_y * vel_y);
	double steer;
	// calculate wheel steer
	if ((vel_x != 0 && vel_y != 0) || fabs(vel_x + vel_y) > 0.1 ) // TODO: add dead zone?
	{
		steer = atan2(vel_y, vel_x); 
	}
	else
	{
		steer = 0;
	}

	// reverse steer when reversing
	int motor_direction = 1;
	int steer_degrees = (int)((steer*4068) / 71);

	if(abs(steer_degrees) <= 90)
	{
		set_angle(90 - steer_degrees);
		motor_direction = 1;
	}
	else if (steer_degrees > 90)
	{
		set_angle(180 - (steer_degrees - 90));
		motor_direction = -1;
	}	
	else
	{
		set_angle( - (steer_degrees + 90) );
		motor_direction = -1;
	}


	// calculate wheel speed
	if(fabs(trans_vel) < 0.1)
	{
		set_speed(0);
	}
	else
	{
		// TODO: don't use hard coded numbers and figure out scaling...
		set_speed(1500 + get_center_speed_bias() + 200*motor_direction*get_speed_scale()*trans_vel);
	}	

	ROS_INFO("Angle: %d	Speed: %d", get_angle(), get_speed());

}	


// Create wheel objects
Wheel wheel_front_right(90, 0, 0.25, 0.25);

void callback(train::base_controllerConfig &config, uint32_t level)
{
	ROS_INFO("Front wheel offset updated to: %d", config.front_wheel_center_bias);
	ROS_INFO("Front wheel velocity scale updated to: %f", config.front_wheel_velocity_scale);

	wheel_front_right.set_center_speed_bias(config.front_wheel_center_bias);
	wheel_front_right.set_speed_scale(config.front_wheel_velocity_scale);
}

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	wheel_front_right.move(twist_aux);
}
	

int main(int argc, char** argv)
{
	ros::init(argc, argv, "base_controller");
	ros::NodeHandle n;


	dynamic_reconfigure::Server<train::base_controllerConfig> server;
	dynamic_reconfigure::Server<train::base_controllerConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	servo_pub = n.advertise<std_msgs::UInt16>("wheel_front_right_speed", 100);
	servo_angle_pub = n.advertise<std_msgs::UInt16>("wheel_front_right_angle", 100);
	ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, cmd_velCallback);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		// publish to topics
		std_msgs::UInt16 msg;
		msg.data = wheel_front_right.get_angle();
		servo_angle_pub.publish(msg);

		msg.data = wheel_front_right.get_speed();
		servo_pub.publish(msg);
		
		ros::spinOnce();
		loop_rate.sleep();

	}
}
