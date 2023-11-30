#include <ros/ros.h>
#include "sensor_msgs/JointState.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "joint_states_pub");
    ros::NodeHandle nh;

    ROS_INFO("joint_states_pub node is Ready!");
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    sensor_msgs::JointState joint_state;
    joint_state.name = { "j1", "j2", "j3", "j4", "j5", "j6",};
    // joint_state.header.frame_id = "aaa";
    joint_state.position = {0.02, -90.02, 0.02, -90.02, 0.02, 0.02};  // 直立状态
    ros::Rate rate(10);  // 单位Hz

    joint_state.header.stamp = ros::Time::now();
    joint_states_pub.publish(joint_state);  // 设置初始姿态
    ROS_INFO("joint_states_pub publish home success!");
    rate.sleep();

    while (ros::ok()) {
        
        for (int i = 0; i < 360; ++i) {
            joint_state.header.stamp = ros::Time::now();
            joint_state.position[0] -= 1;
            joint_states_pub.publish(joint_state);
            rate.sleep();
        }

        for (int i = 0; i < 360; ++i) {
            joint_state.header.stamp = ros::Time::now();
            joint_state.position[0] += 0.1;
            joint_states_pub.publish(joint_state);
            rate.sleep();
        }
    }
    return 0;
}