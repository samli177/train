#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <train/cmd_aux.h>

class TeleopTrain
{
public:
	TeleopTrain();
private:        
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	int linear_x_, linear_y_, angular_z_, angular_y_;
	double l_scale_, a_scale_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;

};

TeleopTrain::TeleopTrain():
	linear_x_(2),
	linear_y_(5),
	angular_z_(3),
	angular_y_(4),
	l_scale_(0.5),
	a_scale_(1) // TODO: clean up
{
	nh_.param("axis_linear_x", linear_x_, linear_x_);
	nh_.param("axis_linear_y", linear_y_, linear_y_);
	nh_.param("axis_angular", angular_z_, angular_z_);
	nh_.param("axis_angular", angular_y_, angular_y_);
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);

	vel_pub_ = nh_.advertise<train::cmd_aux>("cmd_aux", 1, true);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTrain::joyCallback, this);

}

void TeleopTrain::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//geometry_msgs::Twist twist;
	//twist.angular.y = a_scale_*joy->buttons[13];
	//twist.angular.z = a_scale_*joy->buttons[14];
	//twist.linear.x = 1-(l_scale_*joy->axes[linear_x_] + 0.5);
	//twist.linear.y = 1-(l_scale_*joy->axes[linear_y_] + 0.5);
	//vel_pub_.publish(twist);
	
	train::cmd_aux msg;
	if ((1 - (l_scale_*joy->axes[linear_x_] + 0.5)) > 0 )
	{
		if (!joy->buttons[14])
		{
			msg.front_leg_vel =  -(1 - (l_scale_*joy->axes[linear_x_] + 0.5));
		}

		if (!joy->buttons[13])
		{
			msg.back_leg_vel =   -(1 - (l_scale_*joy->axes[linear_x_] + 0.5));
		}
	}
	else
	{
		if (!joy->buttons[14])
		{
			msg.front_leg_vel = 1-(l_scale_*joy->axes[linear_y_] + 0.5);
		}

		if (!joy->buttons[13])
		{
			msg.back_leg_vel = 1-(l_scale_*joy->axes[linear_y_] + 0.5);
		}

	}

	vel_pub_.publish(msg);	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_train");
	TeleopTrain teleop_train;

	ros::spin();
}

