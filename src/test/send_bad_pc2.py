#!/usr/bin/env python

# This program publishes a pointcloud2 which has much shorter data
# than what is described by width, height, and point_step.  RViz
# should catch this and show an error instead of crashing.

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

rospy.init_node( 'bad_pointcloud_publisher' )

pfx = PointField()
pfx.name = 'x'
pfx.offset = 0
pfx.datatype = 7
pfx.count = 1

pfy = PointField()
pfy.name = 'y'
pfy.offset = 4
pfy.datatype = 7
pfy.count = 1

pfz = PointField()
pfz.name = 'z'
pfz.offset = 8
pfz.datatype = 7
pfz.count = 1

pc = PointCloud2()

pc.header.frame_id = "map"
pc.header.stamp = rospy.Time.now()

pc.fields = [ pfx, pfy, pfz ]
pc.data = [ 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0 ]
pc.width = 10
pc.height = 10
pc.point_step = 12
pc.row_step = 120
pc.is_dense = False

pub = rospy.Publisher( 'bad_pointcloud', PointCloud2 )

while not rospy.is_shutdown():
    print( "publishing bad PointCloud2." )
    pub.publish( pc )
    rospy.sleep( 1.0 )
