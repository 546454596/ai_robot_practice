#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped


class OdomToBase():

    def __init__(self):
        self.vx = 0
        self.vy = 0  
        self.vz = 0
        self.vth = 0
        self.x = 0 
        self.y = 0 
        self.z = 0 
        self.ox = 0 
        self.oy = 0 
        self.oz = 0 
        self.ow = 0
        self.th = 0
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.last_time = self.current_time = rospy.Time.now()
        self.delta_x = 0
        self.delta_y = 0
        self.delta_th = 0

        self.vel_sub =rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)
        self.initial_pos_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.pose_callback)     

    def vel_callback(self, msg):
        print('vel_cb')
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        self.delta_x = self.vx * cos(self.th) * dt
        self.delta_y = self.vx * sin(self.th) * dt
        self.delta_th = self.vth * dt

        self.x += self.delta_x
        self.y += self.delta_y
        self.th += self.delta_th

        self.last_time = self.current_time

    def pose_callback(self, posmsg):
        print('pose_cb')
        self.x = posmsg.pose.pose.position.x
        self.y = posmsg.pose.pose.position.y
        self.z = 0
        self.ox = 0
        self.oy = 0
        self.oz = posmsg.pose.pose.orientation.z
        self.ow = posmsg.pose.pose.orientation.w
        (r, p, self.th) = tf.transformations.euler_from_quaternion([self.ox, self.oy, self.oz, self.ow])
        
    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
            self.odom_broadcaster.sendTransform((self.x, self.y, 0.), odom_quat, self.current_time, "base_link", "odom")

            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"

            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

            self.odom_pub.publish(odom)
            r.sleep()


if __name__=='__main__':
    rospy.init_node('odometry_publisher')
    odom_to_baselink = OdomToBase()
    odom_to_baselink.run()
