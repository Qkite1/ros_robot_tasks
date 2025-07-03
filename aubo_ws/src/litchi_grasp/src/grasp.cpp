#include <ros/ros.h>
#include <unistd.h>

#include <iostream>

#include <litchi_grasp/LocatedLitchi.h>
#include <cstdlib>

#include <robotiq_2f_gripper_control/robotiq_2f_gripper_ethercat_client.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <robotiq_ethercat/ethercat_manager.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <ros/ros.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//夹抓控制
class GripperController {
public:
    GripperController(ros::Publisher& gripper_pub) : gripper_pub_(gripper_pub) {
        // 初始化夹爪控制指令
        command_.rACT = 0;
        command_.rGTO = 0;
        command_.rATR = 0;
        command_.rPR = 0;
        command_.rSP = 0;
        command_.rFR = 0;
    }

    void openGripper() {
        // 设置夹爪打开的指令
        command_.rPR = 0;
        command_.rSP = 200; 
        command_.rFR = 150;
        gripper_pub_.publish(command_);
        ros::Duration(1.0).sleep();
    }

    void closeGripper() {
        // 设置夹爪关闭的指令
        command_.rPR = 255;
        command_.rSP = 200; 
        command_.rFR = 150;
        gripper_pub_.publish(command_);
        ros::Duration(1.0).sleep();
    }

    void resetGripper() {
        // 设置夹爪复位的指令
        command_.rACT = 1;
        gripper_pub_.publish(command_);
        ros::Duration(1.0).sleep();
    }

    void activateGripper() {
        // 设置夹爪激活的指令
        command_.rACT = 1;
        command_.rGTO = 1;
        command_.rSP = 255; // 设置夹爪速度
        command_.rFR = 150; // 设置夹爪力度
        // 发布指令
        gripper_pub_.publish(command_);
        ros::Duration(1.0).sleep();
    }

private:
    ros::Publisher& gripper_pub_;
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command_;
};

//机械臂基于末端pose规划
class ArmController{
public:
    ArmController(const std::string& manipulator)
    : move_group(manipulator){
        move_group.setGoalPositionTolerance(0.01);
        move_group.setGoalOrientationTolerance(0.01);
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);
    }

    geometry_msgs::PoseStamped getCurrentPose(const std::string& link = "") const {
        if (link.empty())
            return move_group.getCurrentPose();  // 默认 TCP
        else
            return move_group.getCurrentPose(link);
    }

    void moveToInitialPose(const std::string& target_name){
        ROS_INFO("Moving to initial pose");
        move_group.setNamedTarget(target_name);
        move_group.move();
    }

    void moveToPose(const geometry_msgs::Pose& target_pose){
        ROS_INFO("Moving to target pose");
        move_group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group.move();
        } else {
            ROS_WARN("Failed to plan to target pose.");
        }
    }

    bool isTrajectoryComplete(){
        return !is_moving;
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group;
    bool is_moving;

};


int main(int argc, char ** argv){
    ros::init(argc, argv, "litchi_grasp");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 发布器用于控制夹爪
    ros::Publisher gripper_pub = node.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 10);

    //控制器实例化
    GripperController gripper(gripper_pub);
    ArmController arm("manipulator_i5");

    //夹抓激活
    gripper.resetGripper();
    gripper.activateGripper();
    ros::Duration(1.0).sleep();

    arm.moveToInitialPose("pick");

    geometry_msgs::PoseStamped pose_in_cam;

    while (true) {
    // 1. 调用视觉服务
        ros::ServiceClient client = node.serviceClient<litchi_grasp::LocatedLitchi>("located_litchi");
        litchi_grasp::LocatedLitchi srv;
        if (!client.call(srv)) {
            ROS_ERROR("Failed to call service located_litchi");
            return 1;
        }

        // 2. 获取返回坐标
        pose_in_cam.header.frame_id = "wrist3_Link";
        pose_in_cam.header.stamp = ros::Time(0);
        pose_in_cam.pose.position.x = srv.response.position[0];
        pose_in_cam.pose.position.y = srv.response.position[1];
        pose_in_cam.pose.position.z = srv.response.position[2];

        ROS_INFO("\n📍 检测到坐标: [x=%.3f, y=%.3f, z=%.3f]",
                pose_in_cam.pose.position.x,
                pose_in_cam.pose.position.y,
                pose_in_cam.pose.position.z);

        // 3. 提示用户确认
        std::cout << "是否使用该点进行抓取？按 Enter 确认，按 N/n 重新检测: ";
        std::string user_input;
        std::getline(std::cin, user_input);

        if (user_input != "n" && user_input != "N") {
            ROS_INFO("✅ 用户确认使用该点");
            break;  // 用户确认 → 退出循环
        }

        ROS_WARN("🔁 用户拒绝该点，重新调用视觉服务获取新点...");
        ros::Duration(1.0).sleep();  // 可选：防止过快重复调用
    }


    //设定偏移
    pose_in_cam.pose.position.y -= 0.15;
    pose_in_cam.pose.position.z -= 0.17;
    pose_in_cam.pose.position.x -= 0.04;
    if(pose_in_cam.pose.position.z < 0.0){
        pose_in_cam.pose.position.z = 0.0;
    }

    //反转坐标 摄像头坐标系与末端执行器坐标系
    pose_in_cam.pose.position.x = -pose_in_cam.pose.position.x;
    pose_in_cam.pose.position.y = -pose_in_cam.pose.position.y;

    // 准备 TF 转换器
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::PoseStamped pose_in_base;
    try {
        tfBuffer.transform(pose_in_cam, pose_in_base, "base_link", ros::Duration(1.0));  // 转换到 base_link
        ROS_INFO("Transformed pose: [%.3f, %.3f, %.3f]",
            pose_in_base.pose.position.x,
            pose_in_base.pose.position.y,
            pose_in_base.pose.position.z);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF transform failed: %s", ex.what());
        return 1;
    }

    geometry_msgs::PoseStamped current_pose = arm.getCurrentPose("wrist3_Link");

    // 计算偏移量
    double dx = pose_in_base.pose.position.x - current_pose.pose.position.x;
    double dy = pose_in_base.pose.position.y - current_pose.pose.position.y;
    double dz = pose_in_base.pose.position.z - current_pose.pose.position.z;

    geometry_msgs::Pose goal_pose = current_pose.pose;
    goal_pose.position.x += dx;
    goal_pose.position.y += dy;
    goal_pose.position.z += dz;


    // //固定位置设定
    // litchi_pose.position.x = 0;
    // litchi_pose.position.y = -0.35;
    // litchi_pose.position.z = 0.65;
    // litchi_pose.orientation.x = 0.707;
    // litchi_pose.orientation.y = 0.0;
    // litchi_pose.orientation.z = 0.0;
    // litchi_pose.orientation.w = 0.707;  // 固定单位四元数或保留转换姿态


    // 等待用户确认开始移动（安全防护）
    std::cout << "\n⚠️ 请确认安全，按下回车键继续执行抓取操作..." << std::endl;
    std::cin.get();

    // 控制机械臂移动
    arm.moveToPose(goal_pose);
    gripper.closeGripper();
    ros::Duration(1.0).sleep();

    gripper.openGripper();

    // //移动至收集点
    // pose.position.x=0.3
    // pose.position.y=0.3
    // pose.position.z=0.3
    // pose.orientation.x = 0.0;
    // pose.orientation.y = 0.0;
    // pose.orientation.z = 0.0;
    // pose.orientation.w = 1.0;

    // arm.moveToPose(pose);
    // gripper.openGripper();
    // ros::Duration(1.0).sleep();

    arm.moveToInitialPose("pick");
    ros::Duration(1.0).sleep();
    ROS_INFO(" Done");
    ros::shutdown();
}
