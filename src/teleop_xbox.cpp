#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

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
	linear_x_(1),
	linear_y_(0),
	angular_z_(3),
	angular_y_(4),
	l_scale_(0.3),
	a_scale_(0.3)
{
	nh_.param("axis_linear_x", linear_x_, linear_x_);
	nh_.param("axis_linear_y", linear_y_, linear_y_);
	nh_.param("axis_angular", angular_z_, angular_z_);
	nh_.param("axis_angular", angular_y_, angular_y_);
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTrain::joyCallback, this);
	
}

void TeleopTrain::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;
	twist.angular.y = a_scale_*joy->axes[angular_y_];
	twist.angular.z = a_scale_*joy->axes[angular_z_];
	twist.linear.x = l_scale_*joy->axes[linear_x_];
	twist.linear.y = l_scale_*joy->axes[linear_y_];
	vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_train");
	TeleopTrain teleop_train;

	ros::spin();
}
