#include <math.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_with_circle");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface ur5("manipulator_i5"); // 修改为 i5
    string reference_frame = "base_link";
    ur5.setPoseReferenceFrame(reference_frame);
    ur5.allowReplanning(true);
    ur5.setGoalPositionTolerance(0.001);
    ur5.setGoalOrientationTolerance(0.01);
    ur5.setMaxAccelerationScalingFactor(0.8);
    ur5.setMaxVelocityScalingFactor(0.8);
    
    // 控制机械臂先回到初始化位置
    ur5.setNamedTarget("home");
    ur5.move();
    sleep(1);

    // 设置圆弧路径的参数
    double centerA = 0.070859; // 初始位置的x坐标
    double centerB = 0.16739;  // 初始位置的y坐标
    double radius = 0.15;
    double eef_step = 0.01;
    double jump_threshold = 0.0;
    int maxtries = 200;
    int repeat_count = 3; // 设置重复次数

    for(int repeat = 0; repeat < repeat_count; ++repeat)
    {
        // 创建一个新的目标位姿
        geometry_msgs::Pose target_pose;
        target_pose.orientation.x = 0.70711;
        target_pose.orientation.y = 0;
        target_pose.orientation.z = 0;
        target_pose.orientation.w = 0.70711;
        target_pose.position.x = centerA;
        target_pose.position.y = centerB;
        target_pose.position.z = 0.54716;
        ur5.setPoseTarget(target_pose);
        ur5.move();
        sleep(1);

        // 创建圆弧路径
        vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose);

        for(double theta = 0.0; theta < M_PI*2; theta += 0.01)
        {
            target_pose.position.x = centerA + radius * cos(theta);
            target_pose.position.y = centerB + radius * sin(theta);
            waypoints.push_back(target_pose);
        }

        // 笛卡尔空间下的路径规划
        moveit_msgs::RobotTrajectory trajectory;
        double fraction = 0.0;
        int attempts = 0;
        while(fraction < 1.0 && attempts < maxtries)
        {
            fraction = ur5.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            attempts++;
            if(attempts % 10 == 0)
                ROS_INFO("Still trying after %d attempts...", attempts);
        }

        if(fraction == 1)
        {
            ROS_INFO("Path computed successfully. Moving the arm.");
            // 生成机械臂的运动规划数据
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            // 执行运动
            ur5.execute(plan);
            sleep(0.5);
        }
        else
        {
            ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
        }


    }

        // 控制机械臂先回到初始化位置
    ur5.setNamedTarget("home");
    ur5.move();
    sleep(1);

    ros::shutdown();
    return 0;
}