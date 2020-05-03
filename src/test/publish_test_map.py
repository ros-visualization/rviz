#!/usr/bin/env python

import rospy
import numpy

from nav_msgs.msg import OccupancyGrid

pub = rospy.Publisher('test_map', OccupancyGrid, latch = True)
rospy.init_node('test_map')

_map = OccupancyGrid()
_map.header.stamp = rospy.get_rostime()
_map.header.frame_id = 'map'
_map.info.resolution = .1
_map.info.width = 16
_map.info.height = 16
_map.info.origin.orientation.w = 1
_map.data = [numpy.int8(x) for x in range(256)]

pub.publish( _map )

rospy.spin()
