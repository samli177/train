#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <train/Adc.h>
#include <std_msgs/UInt16.h>
#include <train/wheel_angles.h>

#include <dynamic_reconfigure/server.h>
#include <train/state_publisherConfig.h>
// robot state TODO: make not global...

const double degree = M_PI/180;
double rear_axle = 0, tinc = degree, front_axle=0, angle=0, left_wheel=0, hinc=0.005, turn=0, turnc=0.002, right_wheel=0, right_wheelc=0.001;

double front_axle_scale_=1000, front_axle_offset_=0, rear_axle_scale_=1000, rear_axle_offset_=0;

double front_right_angle, front_left_angle, rear_right_angle, rear_left_angle;

void adcCallback(const train::Adc &adc)
{
	//ROS_INFO("Adc0 value read: %d", adc.adc0);
	front_axle = M_PI*(adc.adc0 - front_axle_offset_)/(front_axle_scale_-front_axle_offset_)-M_PI/4;

	rear_axle = M_PI*(adc.adc1 - rear_axle_offset_)/(rear_axle_scale_-rear_axle_offset_)-M_PI/4;

	//ROS_INFO("Front axle angle %f", front_axle);
}

void wheel_anglesCallback(const train::wheel_angles &wheel_angles)
{
	//ROS_INFO("Servo angle read: %d", angle.data);
	front_right_angle = (wheel_angles.front_right - 90)*degree;
	front_left_angle = (wheel_angles.front_left - 90)*degree;
	rear_right_angle = (wheel_angles.back_right - 90)*degree;
	rear_left_angle = (wheel_angles.back_left - 90)*degree;
	//ROS_INFO("Turning angle %f", front_right_angle);
}

void dynamic_reconfigureCallback( train::state_publisherConfig &config, uint32_t level)
{
	//ROS_INFO("Updating dynamic config");
	front_axle_offset_ = config.front_axle_offset;
	front_axle_scale_ = config.front_axle_scale;
	rear_axle_offset_ = config.rear_axle_offset;
	rear_axle_scale_ = config.rear_axle_scale;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    // subscribe to topics from robot
    ros::Subscriber acd_sub = n.subscribe("adc", 10, adcCallback);
    ros::Subscriber wheel_angles_sub = n.subscribe("wheel_angles", 10, wheel_anglesCallback);

    // set up dynamic reconfigure
    dynamic_reconfigure::Server<train::state_publisherConfig> server;
    dynamic_reconfigure::Server<train::state_publisherConfig>::CallbackType f;

    f = boost::bind(&dynamic_reconfigureCallback, _1, _2);
    server.setCallback(f);

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(10);
        joint_state.position.resize(10);
        joint_state.name[0] ="base_to_front_axle";
        joint_state.position[0] = front_axle;
        joint_state.name[1] ="base_to_rear_axle";
        joint_state.position[1] = rear_axle;
        joint_state.name[2] ="leg_to_front_right_turnleg";
        joint_state.position[2] = front_right_angle;
        joint_state.name[3] ="leg_to_front_left_turnleg";
        joint_state.position[3] = front_left_angle;
        joint_state.name[4] ="leg_to_rear_right_turnleg";
        joint_state.position[4] = rear_right_angle;
        joint_state.name[5] ="leg_to_rear_left_turnleg";
        joint_state.position[5] = rear_left_angle;
        joint_state.name[6] ="front_right_wheel_joint";
        joint_state.position[6] = right_wheel;
        joint_state.name[7] ="front_left_wheel_joint";
        joint_state.position[7] = left_wheel;        
        joint_state.name[8] ="rear_right_wheel_joint";
        joint_state.position[8] = right_wheel;        
        joint_state.name[9] ="rear_left_wheel_joint";
        joint_state.position[9] = left_wheel;

        // update transform
        // (moving in a circle with radius=0.5) EDIT: standing still
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*0.5;
        odom_trans.transform.translation.y = sin(angle)*0.5;
        odom_trans.transform.translation.z = .0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // Create new robot state

        //left_wheel += hinc;
        //if (left_wheel>.2 || left_wheel<0) hinc *= -1;

        //angle += degree/1;
        //turn += turnc;
        //if (turn<-.5 || turn>0) turnc *= -1;
        //right_wheel += right_wheelc;
        //if (right_wheel<-.5 || right_wheel>0) right_wheelc *= -1;

        // This will adjust as needed per iteration
	ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
