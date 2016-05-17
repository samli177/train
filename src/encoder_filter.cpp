#include <ros/ros.h>

//#include <train/Adc.h>
#include <train/wheel_encoder.h>
#include <std_msgs/UInt16.h>

#include <dynamic_reconfigure/server.h>
#include <train/encoder_filterConfig.h>


int low=100, high=400;
double threshold_high=0.6, threshold_low=0.4;
int value = 0;

double v;

int new_loop_rate = 1000;
double weight = 0.01;


void dynamic_reconfigureCallback( train::encoder_filterConfig &config, uint32_t level)
{
	threshold_high = config.front_right_threshold_high;
	threshold_low = config.front_right_threshold_low;
	low = config.front_right_low;
	high = config.front_right_high;
	new_loop_rate = config.loop_rate;	
	weight = config.weight;
}

//void adcCallback(const train::Adc &adc)
//{
//	value = adc.adc0;
//}

void encoderCallback(const train::wheel_encoder &wheel_encoder)
{
	value = wheel_encoder.front_right;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "encoder_filter");
	ros::NodeHandle n;

	// publihser
	ros::Publisher wheel_encoder_pub = n.advertise<std_msgs::UInt16>("wheel_encoder", 1);
	ros::Publisher wheel_speed_pub = n.advertise<std_msgs::UInt16>("wheel_speed", 1);

	// message declarations
	std_msgs::UInt16 msg;
	std_msgs::UInt16 speed_msg;


	// set up dynamic reconfigure
	dynamic_reconfigure::Server<train::encoder_filterConfig> server;
	dynamic_reconfigure::Server<train::encoder_filterConfig>::CallbackType f;

	// subscribe to topics from robot
	// ros::Subscriber adc_sub = n.subscribe("adc", 10, adcCallback);
	ros::Subscriber encoder_sub = n.subscribe("encoder", 10, encoderCallback);

	f = boost::bind(&dynamic_reconfigureCallback, _1, _2);
	server.setCallback(f);

	


	// variables
	double dt;
	double last;
	double v, v_avrage=0;
	ros::Time current_time, last_time;
	last_time = ros::Time::now();
	
	ros::Rate loop_rate(1000);
	

	while (ros::ok())
	{
		loop_rate = new_loop_rate;
		// send the filtered value 
		v = 0;
		if ( value > (low + (high-low)*threshold_high))
		{
			if( last == low)
			{
				current_time = ros::Time::now();
				dt = (current_time - last_time).toSec();
				last_time = current_time;	
				v = 1/dt;
			}

			msg.data = high;
			last = high;
		} else if ( value < (low + (high-low)*threshold_low))
		{
			if( last == high ) 
			{
				current_time = ros::Time::now();
				dt = (current_time - last_time).toSec();
				last_time = current_time;	
				v = 1/dt;

			}
			msg.data = low;
			last = low;
		}
		v_avrage = v_avrage + weight*(v - v_avrage);
		speed_msg.data = 100*v_avrage;
		//ROS_INFO("v_avrage: %f", v_avrage);

		wheel_encoder_pub.publish(msg);
		wheel_speed_pub.publish(speed_msg);
		ros::spinOnce(); 
		loop_rate.sleep();
	}
}
