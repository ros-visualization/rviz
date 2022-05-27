#!/usr/bin/env python

from __future__ import print_function

import rospy
import math
import numpy as np

import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

width = 100
height = 100


def create_pc(t):
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
    ]

    header = Header()
    header.frame_id = "base_link"
    header.stamp = rospy.Time.now()

    # concentric waves
    x, y = np.meshgrid(np.linspace(-2, 2, width), np.linspace(-2, 2, height))
    z = 0.2 * np.sin(3 * np.sqrt(x**2 + y**2) - t)
    points = np.array([x, y, z, z]).reshape(4, -1).T

    return point_cloud2.create_cloud(header, fields, points)


if __name__ == "__main__":
    rospy.init_node("send_tf_v2")

    # transform identical to rot
    tf_static = TransformStamped()
    tf_static.header.stamp = rospy.Time.now()
    tf_static.header.frame_id = "rot"
    tf_static.child_frame_id = "rot2"
    tf_static.transform.rotation.w = 1
    br_static = tf2_ros.StaticTransformBroadcaster()
    br_static.sendTransform(tf_static)

    br = tf2_ros.TransformBroadcaster()
    fast_pub = rospy.Publisher("fast", PointCloud2, queue_size=10)
    slow_pub = rospy.Publisher("slow", PointCloud2, queue_size=1)
    old = create_pc(0.0)

    # base_link moving linearly back and forth
    tf_base = TransformStamped()
    tf_base.header.frame_id = "map"
    tf_base.child_frame_id = "base_link"
    tf_base.transform.rotation.w = 1

    # transform rotating about base_link
    tf_rot = TransformStamped()
    tf_rot.header.frame_id = "base_link"
    tf_rot.child_frame_id = "rot"
    tf_rot.transform.rotation.w = 1

    tf_slow = TransformStamped()
    tf_slow.header.frame_id = "base_link"
    tf_slow.child_frame_id = "slow"
    tf_slow.transform.translation.z = 1
    tf_slow.transform.rotation.w = 1

    fast_rate = rospy.Rate(100)
    slow_rate = rospy.Rate(1)
    frequency = 0.1
    radius = 2
    while not rospy.is_shutdown():
        tf_base.header.stamp = rospy.Time.now()
        t = 2 * math.pi * tf_base.header.stamp.to_sec() * frequency
        tf_base.transform.translation.x = 10 * math.cos(t)
        br.sendTransform(tf_base)

        tf_rot.header.stamp = tf_base.header.stamp
        tf_rot.transform.translation.x = radius * math.cos(2 * t)
        tf_rot.transform.translation.y = radius * math.sin(2 * t)
        tf_rot.transform.translation.z = 0
        br.sendTransform(tf_rot)

        pc = create_pc(2 * t)
        fast_pub.publish(pc)
        fast_rate.sleep()
        print(".", end="")
        if slow_rate.remaining() < rospy.Duration():
            # publish slow TF
            tf_slow.header.stamp = old.header.stamp
            br.sendTransform(tf_slow)
            # re-publish old PC
            slow_pub.publish(old)
            old = pc  # store current PC
            slow_rate.last_time = rospy.Time.now()
            print("+")
