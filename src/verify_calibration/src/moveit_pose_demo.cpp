#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "moveit_pose_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // 创建对象arm连接到xarm规划组
    moveit::planning_interface::MoveGroupInterface arm("fr5_arm");
    // 获取xarm规划组的规划参考坐标系
    std::string planning_frame = arm.getPlanningFrame();
    ROS_INFO_STREAM("Planning frame : " << planning_frame);
    // 获取末端执行器的link
    std::string eef_link = arm.getEndEffectorLink();
    ROS_INFO_STREAM("End effector link : " << eef_link);
    // 若allow_replanning()参数为True，则MoveIt!在一次规划失败后进行重新规划
    arm.allowReplanning(true);
    // 设置运动到目标时的位置(单位：米)和姿态的容忍误差(单位：弧度)
    arm.setGoalPositionTolerance(0.02);
    arm.setGoalOrientationTolerance(0.03);
    // 设置一个比例因子以选择性地降低最大关节速度限制，可取值为(0,1]
    arm.setMaxVelocityScalingFactor(0.02);
    arm.setMaxAccelerationScalingFactor(0.02);

    ROS_INFO("Moving to pose: Handeye_Calibration---");
    arm.setNamedTarget("Handeye_Calibration");
//    arm.move();

    // 使用plan()进行运动规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("moveit_pose_demo", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");
    // 若规划成功,则使用execute()执行规划出的轨迹
    if (success) {
        arm.execute(plan);
    }
    ros::shutdown();
    return 0;
}

