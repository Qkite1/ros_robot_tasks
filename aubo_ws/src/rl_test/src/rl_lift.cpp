#include <ros/ros.h>
#include <unistd.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>

#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>


#include <cstdlib>

#include <robotiq_2f_gripper_control/robotiq_2f_gripper_ethercat_client.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <robotiq_ethercat/ethercat_manager.h>

using namespace std;


/* ========================================================= */
/* +++ ADD +++ */
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <aubo_msgs/SetIO.h>
#include <aubo_msgs/SetIORequest.h>
#include <aubo_msgs/SetIOResponse.h>
#include <std_msgs/Float32MultiArray.h>
/* ========================================================= */

#include <ros/ros.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

//裁剪函数
double clipValue(double value, double min_value, double max_value){
    if(value < min_value){
        return min_value;
    }else if (value > max_value){
        return max_value;
    }else {
        return value;
    }
}
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
        command_.rPR = 150;
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

//机械臂基于关节角度规划
class ArmController{
public:
    ArmController(const std::string& manipulator_i5)
    : move_group(manipulator_i5){
        move_group.setGoalPositionTolerance(0.01);
        move_group.setGoalOrientationTolerance(0.01);
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);
    }

    void moveToInitialPose(){
        ROS_INFO("Moving to initial pose");
        std::vector<double> initial_joint_positions = {1.204, -0.244, -2.478, -0.663, -1.571, 1.222};
        move_group.setJointValueTarget(initial_joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group.plan(plan);
        move_group.move();
    }

    void moveToJointPositions(const std::vector<double>& joint_positions){
        ROS_INFO("Moving to target joint positions");
        move_group.setJointValueTarget(joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group.plan(plan);
        move_group.move();
    }

    bool isTrajectoryComplete(){
        return !is_moving;
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group;
    bool is_moving;

};

//末端数据获取
class EndEffectorPose{
public:
    EndEffectorPose() : listener(){}

    std::vector<double> getEEFData(){
        std::vector<double> eef_data(7, 0.0);

        try{
            tf::StampedTransform transform;
            listener.lookupTransform("/world", "/ee_Link", ros::Time(0), transform);
            //eef_data[0]=-transform.getOrigin().x();
            //eef_data[0]=-transform.getOrigin().y();
            eef_data[0]=-transform.getOrigin().y()-0.56;
            eef_data[1]=transform.getOrigin().x();
            eef_data[2]=transform.getOrigin().z();
            //仿真场景中机械臂坐标轴偏离原点x轴负方向0.56。且x轴为实际场景-y，y轴为实际场景x。
            tf::Quaternion q = transform.getRotation();
            eef_data[3] = q.x();
            eef_data[4] = q.y();
            eef_data[5] = q.z();
            eef_data[6] = q.w();
        }catch (tf::TransformException &ex){
            ROS_ERROR("%s", ex.what());
        }
        return eef_data;
    }
private:
    tf::TransformListener listener;
};

//机械臂关节数据获取
class JointData{
public:
    JointData() : previous_time(0){}

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
        std::vector<double> joint_positions = msg->position;

        if(previous_joint_positions.empty()){
            previous_joint_positions = joint_positions;
            previous_time = std::time(nullptr);
            return;
        }

        double current_time = std::time(nullptr);
        double delta_time = current_time - previous_time;
        if(delta_time == 0){
            return;
        }

        joint_velocities.resize(joint_positions.size());
        for(size_t i =0; i<joint_positions.size(); ++i){
            joint_velocities[i] = (joint_positions[i] - previous_joint_positions[i])/delta_time;
        }

        previous_joint_positions = joint_positions;
        previous_time = current_time;

        joint_pos_cos.resize(joint_positions.size());
        joint_pos_sin.resize(joint_positions.size());
        for(size_t i=0; i<joint_positions.size(); ++i){
            joint_pos_cos[i] = std::cos(joint_positions[i]);
            joint_pos_sin[i] = std::sin(joint_positions[i]); 
        }
    }
    std::vector<double> getJointData() const{
        std::vector<double> joint_data;
        joint_data.insert(joint_data.end(), joint_pos_cos.begin(), joint_pos_cos.end());
        joint_data.insert(joint_data.end(), joint_pos_sin.begin(), joint_pos_sin.end());
        joint_data.insert(joint_data.end(), joint_velocities.begin(), joint_velocities.end());
        return joint_data;
    }
    std::vector<double> getJointPos() const{
        return previous_joint_positions;
    };
private:
    std::vector<double> previous_joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_pos_cos;
    std::vector<double> joint_pos_sin;
    double previous_time;
};

std::vector<double> action;
void actionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
        //处理动作数据
        ROS_INFO("Received action data size: %zu", msg->data.size());
        for (const auto& val : msg->data) {
            ROS_INFO("Received value: %f", val);
            };
        action.clear();
        action.insert(action.end(), msg->data.begin(), msg->data.end());
};

int main(int argc, char ** argv){
    ros::init(argc, argv, "rl_test");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator_i5";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.allowReplanning(true);	// 允许重新规划

    ros::Publisher obs_pub = node.advertise<std_msgs::Float32MultiArray>("obs_data_topic", 1);

    ros::Publisher gripper_pub = node.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 1);
    GripperController gripper_controller(gripper_pub);
    ArmController arm_controller("manipulator_i5");
    EndEffectorPose eefPose;
    JointData jointdata;
    ros::Subscriber joint_sub = node.subscribe("/joint_states", 10, &JointData::jointStatesCallback, &jointdata);
    ros::Subscriber action_sub = node.subscribe("action_topic", 10, actionCallback);
    std::vector<double> object = { };
    //{-0.022, -0.011, 0.519, 0.0. 0.0, 0.56, 0.82}
    std::vector<double> gripper_qpos = {0.039, -0.039};

    gripper_controller.resetGripper();
    gripper_controller.activateGripper();

    arm_controller.moveToInitialPose();

    int validation_steps = 100;//验证500步
    int step_count = 0;

    const int accumulation_steps = 3;
    int accumulation_count = 0;
    std::vector<double> accumulated_action(3, 0.0);

    bool tra_active = false;
    bool gripper_close = false;
    ros::Rate rate(5);
    while(ros::ok() && step_count < validation_steps){

        std::vector<double> eef = eefPose.getEEFData();
        std::vector<double> joint = jointdata.getJointData();
        std::vector<double> joint_positions = jointdata.getJointPos();
       
        std::vector<double> diff(3);
        diff[0] = eef[0] - object[0];
        diff[1] = eef[1] - object[1];
        diff[2] = eef[2] - object[2];

        std::vector<double> obs_data;
        obs_data.insert(obs_data.end(), object.begin(), object.end());
        obs_data.insert(obs_data.end(), diff.begin(), diff.end());
        obs_data.insert(obs_data.end(), joint.begin(), joint.end());
        obs_data.insert(obs_data.end(), eef.begin(), eef.end());
        //if(gripper_value>= 0.20){
        //    gripper_qpos = {0.022, -0.022};
        //}
        obs_data.insert(obs_data.end(), gripper_qpos.begin(), gripper_qpos.end());

        
        std_msgs::Float32MultiArray obs_msg;
        obs_msg.data.clear();
        obs_msg.data.insert(obs_msg.data.end(), obs_data.begin(), obs_data.end());
        obs_pub.publish(obs_msg);

            // 添加强制逻辑：检测机械臂末端执行器的高度
        //if (eef[2] < 0.35) {
        //    ROS_ERROR("Task failed: End-effector height is below 0.5. Breaking the loop.");
        //   break;
        //}

        if(action.size()>=3){

            if(tra_active){
                ROS_INFO("Checking if trajectory is complete...");  
                if(arm_controller.isTrajectoryComplete()){
                    ROS_INFO("Trajectory is complete, disabling tra_active.");  
                    tra_active = false;
                    ros::Duration(0.3).sleep();
                }
            }  

            //判断任务成功
            if(gripper_close &&  eef[2] >=  0.55){ //action[6] == 1 && eef[2] > 0.6
                ROS_INFO("Task completed, breaking the loop.");
                break;
            }

            for (size_t i = 0; i < action.size() - 1; ++i){
                accumulated_action[i] += action[i];
            }
            accumulation_count++;
            if(accumulation_count>= accumulation_steps){
                std::vector<double> target_position = {eef[0]+accumulated_action[0], eef[1]+accumulated_action[1], eef[2]+accumulated_action[2]};
                double min_z_value = 0.47;
                double max_z_value = 0.58;
                target_position[2] = clipValue(target_position[2], min_z_value, max_z_value);
                target_position[0] = clipValue(target_position[0], 0, 0.2);
                target_position[1] = clipValue(target_position[0], 0, 0.2);
                //ROS_INFO("The second element of target_position is: %f", target_position[0]);
                
                geometry_msgs::Pose target_pose;
                target_pose.position.x = target_position[1];
                target_pose.position.y = -(target_position[0] + 0.56);
                target_pose.position.z = target_position[2];
                target_pose.orientation = move_group.getCurrentPose().pose.orientation;

                move_group.setPoseTarget(target_pose);
                move_group.move();
                tra_active = true;
                
                accumulated_action.assign(3, 0.0);
                accumulation_count = 0;
            }
            
           
           ROS_INFO("Current gripper_value: %f", action[3]);
            if(action[3] >= 0.99){
                gripper_controller.closeGripper();
                gripper_close = true;
            }
        

        step_count++;
        rate.sleep();
        }
    }
    ros::Duration(1.5).sleep();
    gripper_controller.openGripper();
    arm_controller.moveToInitialPose();
}
