#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

#include "frcobot_hw/frcobot_hw.h"
#include "frcobot_hw/FRRobot.h"
#include "frcobot_hw/RobotError.h"
#include "frcobot_hw/RobotTypes.h"

#include "xmlrpc-c/base.h"
#include "xmlrpc-c/client.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "joint_states_pub");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::NodeHandle nh;

    ROS_DEBUG("joint_states_pub node is Ready!");
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    sensor_msgs::JointState joint_state;
    joint_state.name = { "j1", "j2", "j3", "j4", "j5", "j6",};
    // joint_state.header.frame_id = "aaa";
    joint_state.position = {0.02, -90.02, 0.02, -90.02, 0.02, 0.02};  // 直立状态
    ros::Rate rate(10);  // 单位Hz

    joint_state.header.stamp = ros::Time::now();
    joint_states_pub.publish(joint_state);  // 设置初始姿态
    ROS_DEBUG("joint_states_pub publish home success!");
    rate.sleep();

    FRRobot robot;                 //Instantiate the robot object
    robot.RPC("192.168.58.2");     //Establish a communication connection with the robot controller

    char ip[64]="";
    char version[64] = "";
    uint8_t state;
    int id;
    int  type = 0;
    DescPose flange, desc_pose, pre_pose, post_pose, offset_pose;
    JointPos j_rad, j_pose_inver;
    ExaxisPos epos;

    int tool = 0;
    int user = 0;
    float vel = 100.0;
    float acc = 100.0;
    float ovl = 100.0;
    float blendT = 0.0;
    float blendR = 0.0;
    uint8_t flag = 0;
    uint8_t search = 0;

    uint8_t status_0 = 0; 
    uint8_t status_1 = 1; 
    uint8_t smooth = 0;
    uint8_t block = 0;

    memset(&epos, 0, sizeof(ExaxisPos));
    memset(&offset_pose, 0, sizeof(DescPose));

    robot.GetSDKVersion(version);
    ROS_DEBUG("SDK version:%s\n", version);
    robot.GetControllerIP(ip);
    ROS_DEBUG("controller ip:%s\n", ip);

    // auto mode
    robot.Mode(0);
    sleep(1);

    robot.GetActualToolFlangePose(flag, &flange);
    ROS_DEBUG("tcp pose : %f, %f, %f, %f, %f, %f\n", flange.tran.x, flange.tran.y,  flange.tran.z, flange.rpy.rx, flange.rpy.ry, flange.rpy.rz);

    robot.GetActualJointPosRadian(flag, &j_rad);
    ROS_DEBUG("joint pos rad:%f,%f,%f,%f,%f,%f\n", j_rad.jPos[0],j_rad.jPos[1],j_rad.jPos[2],j_rad.jPos[3],j_rad.jPos[4],j_rad.jPos[5]);

    robot.GetActualTCPNum(flag, &id);
    ROS_DEBUG("tcp num:%d\n", id);

    int  ret = robot.GetInverseKin(type, &flange, -1, &j_pose_inver);
    ROS_DEBUG("joint pos InverseKin:%d,%f,%f,%f,%f,%f,%f\n", ret, j_pose_inver.jPos[0],j_pose_inver.jPos[1],j_pose_inver.jPos[2],j_pose_inver.jPos[3],j_pose_inver.jPos[4],j_pose_inver.jPos[5]);

    robot.GetForwardKin(&j_pose_inver, &desc_pose);
    ROS_DEBUG("desc_pose tcp pose : %f, %f, %f, %f, %f, %f\n", desc_pose.tran.x, desc_pose.tran.y,  desc_pose.tran.z, desc_pose.rpy.rx, desc_pose.rpy.ry, desc_pose.rpy.rz);


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