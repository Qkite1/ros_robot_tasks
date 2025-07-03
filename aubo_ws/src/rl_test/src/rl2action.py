#!/usr/bin/env python3
# -*-coding: utf-8 -*-

import rospy
from stable_baselines3 import PPO
from std_msgs.msg import Float32MultiArray
import numpy as np
import d3rlpy
from d3rlpy.algos import BC, IQL, IDQL, AC

rospy.init_node('rl_mode_node')

model = d3rlpy.algos.IQL.from_json("src/rl_test/src/train/iql/params.json")
model.load_model("src/rl_test/src/train/iql/model_300000.pt")

action_pub = rospy.Publisher("action_topic", Float32MultiArray, queue_size=10)

input_max = 1.0
input_min = -1.0
output_max = 0.04
output_min = -0.04

def generate_action(obs_data):
    obs_batch = obs_data[np.newaxis, :]
    action= model.predict(obs_batch)
    action = action[0]
    action[ :3 ] = np.clip(action[ :3 ], output_min, output_max)
    #scaled_action = output_min + (output_max - output_min)*(action - input_min)/(input_max - input_min)
    rospy.loginfo(f"generated action:{action}")
    return action

def callback(data):

    obs_data = np.array(data.data)
    action = generate_action(obs_data)

    action_msg = Float32MultiArray()
    action_msg.data = action
    action_pub.publish(action_msg)
    rospy.loginfo(f"publish action: {action_msg.data}")

rospy.Subscriber("obs_data_topic",     Float32MultiArray,  callback )
rospy.sleep(2)

rospy.spin()
