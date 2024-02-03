#!/usr/bin/env python

# This program publishes a pointcloud2 to test rviz rendering

from __future__ import print_function

import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

import math
import time
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

RATE = 30
width = 100
height = 100


def generate_point_cloud():
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
    ]

    header = Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()

    x, y = np.meshgrid(np.linspace(-2, 2, width), np.linspace(-2, 2, height))
    z = 0.5 * np.sin(2 * x) * np.sin(3 * y)
    points = np.array([x, y, z, z]).reshape(4, -1).T

    return point_cloud2.create_cloud(header, fields, points)


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


if __name__ == "__main__":
    rospy.init_node("point_cloud_test")

    tf_broadcaster = TransformBroadcaster()

    vis_pub = rospy.Publisher("robot/marker", Marker, queue_size=5)
    pc_pub = rospy.Publisher("points2", PointCloud2, queue_size=5)
    pc = generate_point_cloud()

    i = 0
    rate = rospy.Rate(RATE)
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()

        if i % RATE == 0:
            # publish just once per second
            pc.header.stamp = now
            pc_pub.publish(pc)
        i += 1

        robot_x, robot_y, robot_yaw = draw_eight((now - start).to_sec())
        q = quaternion_from_euler(0, 0, robot_yaw)

        tf = TransformStamped()
        tf.header.frame_id = "map"
        tf.header.stamp = now
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = robot_x
        tf.transform.translation.y = robot_y
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        tf_broadcaster.sendTransform(tf)

        display_dummy_robot(now)

        rate.sleep()
