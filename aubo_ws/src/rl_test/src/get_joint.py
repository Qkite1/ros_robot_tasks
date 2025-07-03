#!/usr/bin/env python2.7
# -*-coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
import math
import time

# 记录上一次的位置和时间戳
previous_joint_positions = None
previous_time = None

def joint_states_callback(msg):
    global previous_joint_positions, previous_time
    
    # 获取当前时间
    current_time = time.time()

    # 获取关节位置
    joint_positions = msg.position
    
    # 如果之前没有记录过位置和时间，则初始化
    if previous_joint_positions is None:
        previous_joint_positions = joint_positions
        previous_time = current_time
        return

    # 计算时间差
    delta_time = current_time - previous_time
    if delta_time == 0:
        return

    # 计算关节速度（关节位置的变化除以时间差）
    joint_velocities = [(current - previous) / delta_time for current, previous in zip(joint_positions, previous_joint_positions)]
    
    # 更新之前的位置和时间
    previous_joint_positions = joint_positions
    previous_time = current_time

    # 计算 joint pos cos 和 joint pos sin
    joint_pos_cos = [math.cos(pos) for pos in joint_positions]
    joint_pos_sin = [math.sin(pos) for pos in joint_positions]

    # 输出结果
    print("Calculated Joint Velocities (radians/s):", joint_velocities)
    print("Joint Positions Cos:", joint_pos_cos)
    print("Joint Positions Sin:", joint_pos_sin)

# 初始化 ROS 节点
rospy.init_node('joint_state_listener')

# 订阅 /joint_states 话题
rospy.Subscriber("/joint_states", JointState, joint_states_callback)

# 保持节点运行
rospy.spin()
