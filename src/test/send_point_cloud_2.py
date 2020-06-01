#!/usr/bin/env python

# This program publishes a pointcloud2 to test rviz rendering

from __future__ import print_function

import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

width = 100
height = 100

def publishPC2(count):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1)]

    header = Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()
    
    x, y = np.meshgrid(np.linspace(-2,2,width), np.linspace(-2,2,height))
    z = 0.5 * np.sin(2*x - count/10.0) * np.sin(2*y)
    points = np.array([x,y,z,z]).reshape(4,-1).T

    pc2 = point_cloud2.create_cloud(header, fields, points)
    pub.publish(pc2)

if __name__ == '__main__':
    rospy.init_node('pc2_publisher')
    pub = rospy.Publisher('points2', PointCloud2, queue_size=100)
    rate = rospy.Rate(10)

    count = 0
    while not rospy.is_shutdown():
        publishPC2(count)
        count += 1
        rate.sleep()
