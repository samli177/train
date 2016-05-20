#include <ros/ros.h> 
#include <train/wheel_encoder.h> 
#include <train/wheel_vel.h>
#include <std_msgs/UInt16.h>
#include <dynamic_reconfigure/server.h> 
#include <train/encoder_filterConfig.h>

// parameters
double DIAMETER = 0.0804;
double CIRCUMFERENCE = DIAMETER*M_PI;
int TICKS_PER_REVOLUTION = 60;
double DISTANCE_PER_TICK = CIRCUMFERENCE/TICKS_PER_REVOLUTION;
int TICK_DIV = 500;


int low=100, high=400;
double threshold_high=0.6, threshold_low=0.4;
int value = 0;

double v;

int new_loop_rate = 1000;
double weight = 0.01;
double scale = 1;

class Wheel
{
public:
	Wheel();

	int low_, high_;
	double threshold_high_, threshold_low_;
	int raw_value_;

	double dt_, last_;
	double v_, v_avrage_;
	int filtered_;
	long ticks_;

	double v_ticks_;
	int count_;
	long last_ticks_;

	void update_config(int low, int high, double threshold_high, double threshold_low);
	void update();

	void update_ticks();
	
	ros::Time current_time_, last_time_, current_time_ticks_, last_time_ticks_;
	
	// message declaration
	std_msgs::UInt16 msg_;
	std_msgs::UInt16 speed_msg_;

};

Wheel::Wheel()
{
	low_ = 100;
	high_ = 400;
	threshold_high_ = 0.6;
	threshold_low_ = 0.4;
	raw_value_ = 0;

	v_ = 0;
	v_avrage_ = 0;

	dt_ = 1;

	filtered_ = low_;

	ticks_ = 0; } 
	
void Wheel::update_config(int low, int high, double threshold_high, double threshold_low) 
{
	low_ = low;
	high_ = high;
	threshold_high_ = threshold_high;
	threshold_low_ = threshold_low;

	v_ticks_ = 0;
	count_ = 0;
	last_ticks_ = 0;
	

	last_time_ticks_ = ros::Time::now();
}

void Wheel::update_ticks()
{
	if (count_ > TICK_DIV)
	{
		current_time_ticks_ = ros::Time::now();
		v_ticks_ = DISTANCE_PER_TICK*(ticks_-last_ticks_)*(current_time_ticks_ - last_time_ticks_).toSec();
		last_time_ticks_ = current_time_ticks_;
		last_ticks_ = ticks_;
		//ROS_INFO("v_ticks: %f", v_ticks_);
		count_ = 0;
	}
	count_++;
}

void Wheel::update()
{
	// variables
	v_ = 0;

	if ( raw_value_ > (low_ + (high_-low_)*threshold_high_))
	{
		if( last_ == low_)
		{
			current_time_ = ros::Time::now();
			dt_ = (current_time_ - last_time_).toSec();
			last_time_ = current_time_;	
			v_ = DISTANCE_PER_TICK/dt_; 
			ticks_++;
			//ROS_INFO("ticks: %d", ticks_); 
			//ROS_INFO("distance: %f", ticks_*DISTANCE_PER_TICK);
		}

		filtered_ = high_;
		msg_.data = high_;
		last_ = high_;
	} else if ( raw_value_ < (low_ + (high_-low_)*threshold_low_))
	{
		if( last_ == high_ ) 
		{
			current_time_ = ros::Time::now();
			dt_ = (current_time_ - last_time_).toSec();
			last_time_ = current_time_;	
			v_ = DISTANCE_PER_TICK/dt_;
			ticks_++;
			//ROS_INFO("ticks: %d", ticks_); }
			filtered_ = low_;
			msg_.data = low_;
			last_ = low_;
		}
	}
	v_avrage_ = v_avrage_ + weight*(v_ - v_avrage_);
	speed_msg_.data = 100*v_avrage_;
	//ROS_INFO("v_avrage: %f", v_avrage);
}
	

// objects
Wheel wheel_front_right;
Wheel wheel_front_left;
Wheel wheel_rear_right;
Wheel wheel_rear_left;

void dynamic_reconfigureCallback( train::encoder_filterConfig &config, uint32_t level)
{
	wheel_front_right.threshold_high_ = config.front_right_threshold_high;
	wheel_front_right.threshold_low_ = config.front_right_threshold_low;
	wheel_front_right.low_ = config.front_right_low;
	wheel_front_right.high_ = config.front_right_high;

	wheel_front_left.threshold_high_ = config.front_left_threshold_high;
	wheel_front_left.threshold_low_ = config.front_left_threshold_low;
	wheel_front_left.low_ = config.front_left_low;
	wheel_front_left.high_ = config.front_left_high;

	wheel_rear_right.threshold_high_ = config.rear_right_threshold_high;
	wheel_rear_right.threshold_low_ = config.rear_right_threshold_low;
	wheel_rear_right.low_ = config.rear_right_low;
	wheel_rear_right.high_ = config.rear_right_high;

	wheel_rear_left.threshold_high_ = config.rear_left_threshold_high;
	wheel_rear_left.threshold_low_ = config.rear_left_threshold_low;
	wheel_rear_left.low_ = config.rear_left_low;
	wheel_rear_left.high_ = config.rear_left_high;

	weight = config.weight;
	new_loop_rate = config.loop_rate;
	scale = config.vel_scale;

}


void encoderCallback(const train::wheel_encoder &wheel_encoder)
{
	wheel_front_right.raw_value_ = wheel_encoder.front_right;
	wheel_front_left.raw_value_ = wheel_encoder.front_left;
	wheel_rear_right.raw_value_ = wheel_encoder.rear_right;
	wheel_rear_left.raw_value_ = wheel_encoder.rear_left;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "encoder_filter");
	ros::NodeHandle n;

	// publihser
	ros::Publisher wheel_encoder_pub = n.advertise<std_msgs::UInt16>("wheel_encoder", 1);
	ros::Publisher wheel_speed_pub = n.advertise<std_msgs::UInt16>("wheel_speed", 1);
	ros::Publisher wheel_encoders_filtered_pub = n.advertise<train::wheel_encoder>("wheel_encoders_filtered", 1);
	ros::Publisher wheel_encoder_vel_pub = n.advertise<train::wheel_vel>("wheel_encoder_vel", 1);
	ros::Publisher wheel_encoder_vel_ticks_pub = n.advertise<train::wheel_vel>("wheel_encoder_vel_ticks", 1);

	// set up dynamic reconfigure
	dynamic_reconfigure::Server<train::encoder_filterConfig> server;
	dynamic_reconfigure::Server<train::encoder_filterConfig>::CallbackType f;

	// subscribe to topics from robot
	// ros::Subscriber adc_sub = n.subscribe("adc", 10, adcCallback);
	ros::Subscriber encoder_sub = n.subscribe("encoder", 10, encoderCallback);

	// messages
	train::wheel_encoder msg_filtered; 
	train::wheel_vel msg_vel;
	train::wheel_vel msg_vel_ticks;

	f = boost::bind(&dynamic_reconfigureCallback, _1, _2);
	server.setCallback(f);

	ros::Rate loop_rate(1000);
	
	while (ros::ok())
	{

		loop_rate = new_loop_rate;

		// front right
		wheel_front_right.update();
		msg_filtered.front_right = wheel_front_right.filtered_;
		msg_vel.front_right = scale*wheel_front_right.v_avrage_;
		wheel_front_right.update_ticks();
		msg_vel_ticks.front_right = wheel_front_right.v_ticks_;
		
		// front left	
		wheel_front_left.update();
		msg_filtered.front_left = wheel_front_left.filtered_;
		msg_vel.front_left = scale*wheel_front_left.v_avrage_;
		wheel_front_left.update_ticks();
		msg_vel_ticks.front_left = wheel_front_left.v_ticks_;
		
		// rear right
		wheel_rear_right.update();
		msg_filtered.rear_right = wheel_rear_right.filtered_;
		msg_vel.back_right = scale*wheel_rear_right.v_avrage_;
		wheel_rear_right.update_ticks();
		msg_vel_ticks.back_right = wheel_front_right.v_ticks_;
	
		// rear left	
		wheel_rear_left.update();
		msg_filtered.rear_left = wheel_rear_left.filtered_;
		msg_vel.back_left = scale*wheel_rear_left.v_avrage_;
		wheel_rear_left.update_ticks();
		msg_vel_ticks.back_left = wheel_rear_left.v_ticks_;
	
		// publish messages	
		wheel_encoders_filtered_pub.publish(msg_filtered);
		wheel_encoder_vel_pub.publish(msg_vel);

		// wheel_encoder_pub.publish(wheel_front_right.msg_);
		// wheel_speed_pub.publish(wheel_front_right.speed_msg_);

		wheel_encoder_vel_ticks_pub.publish(msg_vel_ticks);

		ros::spinOnce(); 
		loop_rate.sleep();
	}
}
