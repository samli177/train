#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double rear_axle = 0, tinc = degree, front_axle=0, angle=0, left_wheel=0, hinc=0.005, turn=0, turnc=0.002, right_wheel=0, right_wheelc=0.001;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

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
        joint_state.position[2] = turn;
        joint_state.name[3] ="leg_to_front_left_turnleg";
        joint_state.position[3] = turn;
        joint_state.name[4] ="leg_to_rear_right_turnleg";
        joint_state.position[4] = turn;
        joint_state.name[5] ="leg_to_rear_left_turnleg";
        joint_state.position[5] = turn;
        joint_state.name[6] ="front_right_wheel_joint";
        joint_state.position[6] = right_wheel;
        joint_state.name[7] ="front_left_wheel_joint";
        joint_state.position[7] = left_wheel;        
        joint_state.name[8] ="rear_right_wheel_joint";
        joint_state.position[8] = right_wheel;        
        joint_state.name[9] ="rear_left_wheel_joint";
        joint_state.position[9] = left_wheel;

        // update transform
        // (moving in a circle with radius=0.5)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*0.5;
        odom_trans.transform.translation.y = sin(angle)*0.5;
        odom_trans.transform.translation.z = .0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // Create new robot state

        left_wheel += hinc;
        if (left_wheel>.2 || left_wheel<0) hinc *= -1;

        angle += degree/1;
        turn += turnc;
        if (turn<-.5 || turn>0) turnc *= -1;
        right_wheel += right_wheelc;
        if (right_wheel<-.5 || right_wheel>0) right_wheelc *= -1;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
