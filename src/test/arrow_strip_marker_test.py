#!/usr/bin/env python

import rospy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA


FRAME_ID = "map"
rospy.init_node("marker_test")
marker_pub = rospy.Publisher("marker_test", Marker)

def generate_arrows(points, pose, scale, color):
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = FRAME_ID
    m.header.stamp = rospy.Time.now()
    m.ns = "arrow_strip"
    m.id = 0
    # TODO: Enable when https://github.com/ros/common_msgs/pull/190 is merged.
    # m.type = Marker.ARROW_STRIP
    m.points = points
    m.pose = pose
    m.scale = scale
    m.color = color
    return m

def generate_circle(radius, samples):
    points = []
    angle_step = (2 * math.pi) / samples
    for i in range(samples):
        x = radius * math.sin(angle_step * i)
        y = radius * math.cos(angle_step * i)
        points.append(Point(x, y, 0))
    points.append(points[0])
    return points

def to_quaternion(axis, angle):
    return Quaternion(
        axis[0] * math.sin(angle / 2),
        axis[1] * math.sin(angle / 2),
        axis[2] * math.sin(angle / 2),
        math.cos(angle / 2)
    )

points = generate_circle(radius=1, samples=50)
arrows = generate_arrows(
    points=points,
    pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
    scale=Vector3(0.01, 0.05, 0.05),
    color=ColorRGBA(1.0, 0, 0, 1.0)
)
angle = 0

while not rospy.is_shutdown():
    angle = (angle + 1) % 360
    arrows.pose.orientation = to_quaternion([1, 0, 0], (float)(angle) * (math.pi / 180.0))
    marker_pub.publish(arrows)
    rospy.sleep(0.05)
