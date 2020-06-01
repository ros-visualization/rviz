#!/usr/bin/env python

# This program publishes a pointcloud2 to test rviz rendering

import math
import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

rate = 100
moving = True
width = 100
height = 100

def publishPC2(count):
    fields = [ PointField('x', 0, PointField.FLOAT32, 1),
               PointField('y', 4, PointField.FLOAT32, 1),
               PointField('z', 8, PointField.FLOAT32, 1),
               PointField('intensity', 12, PointField.FLOAT32, 1) ]

    header = Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()
    
    i, j = np.meshgrid(range(width), range(height))

    x = i * 4 / width
    y = j * 4 / height
    z = (0.5 * np.sin((i-count)/10.0) * np.sin(j/10.0))

    points = list(zip(x.ravel(), y.ravel(), z.ravel(), z.ravel()))

    if (moving):
        count += 1

    pc2 = point_cloud2.create_cloud(header, fields, points)
    pub.publish( pc2 )

if __name__ == '__main__':
    rospy.init_node( 'pc2_publisher' )
    pub = rospy.Publisher( 'test_cloud', PointCloud2, queue_size=100 )

    count = 0
    while not rospy.is_shutdown():
        count += 1
        publishPC2(count)
        rospy.sleep(1/rate)
