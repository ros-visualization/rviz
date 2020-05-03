#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA


rospy.init_node('marker_test')
marker_pub = rospy.Publisher('marker_test', Marker)

def make_marker(marker_type, scale, r, g, b, a):
    # make a visualization marker array for the occupancy grid
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = 'base_link'
    m.header.stamp = rospy.Time.now()
    m.ns = 'marker_test_%d' % marker_type
    m.id = 0
    m.type = marker_type
    m.pose.orientation.y = 0
    m.pose.orientation.w = 1
    m.scale = scale
    m.color.r = 1.0
    m.color.g = 0.5
    m.color.b = 0.2
    m.color.a = 0.3
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = a
    return m

def make_arrow_points_marker(scale, tail, tip, idnum):
    # make a visualization marker array for the occupancy grid
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = 'base_link'
    m.header.stamp = rospy.Time.now()
    m.ns = 'points_arrows'
    m.id = idnum
    m.type = Marker.ARROW
    m.pose.orientation.y = 0
    m.pose.orientation.w = 1
    m.scale = scale
    m.color.r = 0.2
    m.color.g = 0.5
    m.color.b = 1.0
    m.color.a = 0.3

    m.points = [ tail, tip ]
    return m

while not rospy.is_shutdown():
    rospy.loginfo('Publishing arrow marker')

    #marker_pub.publish(make_arrow_points_marker(Point(0,0,0), Point(2,2,0), 0))
    #marker_pub.publish(make_arrow_points_marker(Point(0,0,0), Point(1,-1,1), 1))
    #marker_pub.publish(make_arrow_points_marker(Point(-1,-1,-1), Point(1,-1,-1), 2))

    #this arrow should look exactly like the other one, except that is
    #is twice a wide in the z direction.
    scale = Vector3(2,4,0.69)
    marker_pub.publish(make_arrow_points_marker(scale,Point(0,0,0), Point(3,0,0), 3))

    scale = Vector3(3,2,1)
    marker_pub.publish(make_marker(Marker.SPHERE,   scale, 1, .5, .2, .3))
    marker_pub.publish(make_marker(Marker.CYLINDER, scale, .5, .2, 1, .3))
    marker_pub.publish(make_marker(Marker.CUBE,     scale, .2, 1, .5, .3))
    marker_pub.publish(make_marker(Marker.ARROW,    scale, 1, 1, 1, .5))
    rospy.sleep(1.0)
