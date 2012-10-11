#!/usr/bin/env python

PACKAGE_NAME = 'rviz'
import roslib; roslib.load_manifest(PACKAGE_NAME)
#import sys, os.path, time
#import numpy as np
#from scipy import linalg
#from matplotlib import pyplot as plt
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA



rospy.init_node('marker_test')
marker_pub = rospy.Publisher('marker_test', Marker)

def make_marker(marker_type, idnum=[0], scale=Vector3(1,1,1)):
    # make a visualization marker array for the occupancy grid
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = '/base_link'
    m.header.stamp = rospy.Time.now()
    m.ns = 'marker_test'
    m.id = idnum[0]
    idnum[0] = idnum[0] + 1
    m.type = marker_type
    m.pose.orientation.y = 0
    m.pose.orientation.w = 1
    m.scale = scale
    m.color.r = 1.0;
    m.color.g = 0.5;
    m.color.b = 0.2;
    m.color.a = 0.3;
    return m

def make_arrow_points_marker(tail, tip, idnum=[0]):
    # make a visualization marker array for the occupancy grid
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = '/base_link'
    m.header.stamp = rospy.Time.now()
    m.ns = 'points_arrows'
    m.id = idnum[0]
    idnum[0] = idnum[0]+1
    m.type = Marker.ARROW
    m.pose.orientation.y = 0
    m.pose.orientation.w = 1
    m.scale.x = 0.5
    m.scale.y = 1.0
    m.scale.z = 0.5
    m.color.r = 0.2
    m.color.g = 0.5
    m.color.b = 1.0
    m.color.a = 0.3

    m.points = [ tail, tip ]
    return m
 
while not rospy.is_shutdown():
    rospy.loginfo('Publishing arrow marker')
    counter = [0]
  
    marker_pub.publish(make_arrow_points_marker(Point(0,0,0), Point(2,2,0), counter))
    marker_pub.publish(make_arrow_points_marker(Point(0,0,0), Point(1,-1,1), counter))
    marker_pub.publish(make_arrow_points_marker(Point(-1,-1,-1), Point(1,-1,-1), counter))
    scale = Vector3(3,2,1)
    marker_pub.publish(make_marker(Marker.SPHERE,   counter, scale))
    marker_pub.publish(make_marker(Marker.CYLINDER, counter, scale))
    marker_pub.publish(make_marker(Marker.CUBE,     counter, scale))
    marker_pub.publish(make_marker(Marker.ARROW,    counter, scale))
    rospy.sleep(1.0)

