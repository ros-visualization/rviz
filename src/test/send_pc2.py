#!/usr/bin/env python

# This program publishes a pointcloud2 to test rviz rendering

import math
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

rospy.init_node( 'pc2_publisher' )
pub = rospy.Publisher( 'test_cloud', PointCloud2, queue_size=10)

rate = 1
moving = True
width = 100
height = 100

def publishPC2():
    print( "publishing test PointCloud2" )
    fields = [ PointField('x', 0, PointField.FLOAT32, 1),
               PointField('y', 4, PointField.FLOAT32, 1),
               PointField('z', 8, PointField.FLOAT32, 1) ]

    header = Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()

    count = 0
    points = []
    for i in range(width):
        for j in range(height):
           x = float(i) * 4 / width
           y = float(j) * 4 / height
           z = 0.1 *  math.sin(float(i)/10.0) * math.sin(float(j)/10.0)
           pt = [x, y, z]
           points.append(pt)

    pc2 = point_cloud2.create_cloud(header, fields, points)
    pub.publish( pc2 )

while not rospy.is_shutdown():
    publishPC2()
    rospy.sleep(rate)
