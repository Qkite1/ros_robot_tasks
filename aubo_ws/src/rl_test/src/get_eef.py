#!/usr/bin/env python2.7
# -*-coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def get_end_effector_pose():
    rospy.init_node('end_effector_pose_listener')
    listener = tf.TransformListener()
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            # 获取末端执行器相对于世界坐标系的位置和姿态
            (trans, rot) = listener.lookupTransform('/world', '/ee_Link', rospy.Time(0))
            
            # 将坐标变换转换为 PoseStamped 消息
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = 'world'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            
            # 打印位置信息和姿态信息
            rospy.loginfo("End-Effector Pose:")
            rospy.loginfo("Position: x=%.2f, y=%.2f, z=%.2f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            rospy.loginfo("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Error: %s", e)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        get_end_effector_pose()
    except rospy.ROSInterruptException:
        pass