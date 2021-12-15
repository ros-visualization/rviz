#!/usr/bin/env python

import math
import time

import rospy
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf2_ros.transform_broadcaster import TransformBroadcaster

RATE = 30


def odom_to_tf(odom_msg):
    tf = TransformStamped()
    tf.header = odom_msg.header
    tf.child_frame_id = odom_msg.child_frame_id
    tf.transform.translation = odom_msg.pose.pose.position
    tf.transform.rotation = odom_msg.pose.pose.orientation
    tf_broadcaster.sendTransform(tf)


def draw_eight(elapsed_time, full_cycle_duration=10, scale=10):
    # lemniscate of Gerono
    progress = (elapsed_time % full_cycle_duration) / full_cycle_duration
    t = -0.5 * math.pi + progress * (2 * math.pi)
    x = math.cos(t) * scale
    y = math.sin(t) * math.cos(t) * scale
    dx_dt = -math.sin(t)
    dy_dt = math.cos(t) * math.cos(t) - math.sin(t) * math.sin(t)
    yaw = math.atan2(dy_dt, dx_dt)
    return x, y, yaw


def display_dummy_robot(stamp):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = stamp
    marker.ns = "robot"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    vis_pub.publish(marker)


rospy.init_node("odometry_test", anonymous=True)

tf_broadcaster = TransformBroadcaster()

odom_pub = rospy.Publisher("robot/odom", Odometry, queue_size=5)
vis_pub = rospy.Publisher("robot/marker", Marker, queue_size=5)

time.sleep(1)

rate = rospy.Rate(RATE)
start = rospy.Time.now()
while not rospy.is_shutdown():
    now = rospy.Time.now()

    robot_x, robot_y, robot_yaw = draw_eight((now - start).to_sec())
    q = quaternion_from_euler(0, 0, robot_yaw)

    odom_robot = Odometry()
    odom_robot.header.stamp = now
    odom_robot.header.frame_id = "world"
    odom_robot.child_frame_id = "base_link"
    odom_robot.pose.pose.position.x = robot_x
    odom_robot.pose.pose.position.y = robot_y
    odom_robot.pose.pose.orientation.x = q[0]
    odom_robot.pose.pose.orientation.y = q[1]
    odom_robot.pose.pose.orientation.z = q[2]
    odom_robot.pose.pose.orientation.w = q[3]

    odom_to_tf(odom_robot)

    odom_pub.publish(odom_robot)

    display_dummy_robot(now)

    rate.sleep()
