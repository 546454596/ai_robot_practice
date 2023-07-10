#!/usr/bin/env python
#coding:utf-8
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import math


def pose_pub(alpha,x_pos,y_pos):

    initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

    pose_msg=PoseWithCovarianceStamped()
    pose_msg.header.frame_id = "map"
    pose_msg.pose.pose.position.x = x_pos
    pose_msg.pose.pose.position.y = y_pos
    pose_msg.pose.covariance[0] = 0.25
    pose_msg.pose.covariance[6 * 1 + 1] = 0.25
    pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942
    pose_msg.pose.pose.orientation.z = math.sin(alpha/2)
    pose_msg.pose.pose.orientation.w = math.cos(alpha/2)

    initial_pose_pub.publish(pose_msg)

if __name__ == '__main__':
    #初始化节点
    rospy.init_node("initialpose_node")
    alpha = -3.1415/4
    x_pos = 17
    y_pos = 5
    rate = rospy.Rate(10) # 10hz  
    while not rospy.is_shutdown():
        pose_pub(alpha,x_pos,y_pos)
        rate.sleep() 

