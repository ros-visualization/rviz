#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
from nav_msgs.msg import OccupancyGrid
import rospy

topic = 'map'
publisher = rospy.Publisher(topic, OccupancyGrid)

rospy.init_node('map_test')

grid = OccupancyGrid()

while not rospy.is_shutdown():

    grid.header.frame_id = "/map"
    grid.header.stamp = rospy.Time.now()
    grid.info.map_load_time = rospy.Time.now()
    grid.info.resolution = 1.0
    grid.info.width = 3
    grid.info.height = 3
    grid.info.origin.orientation.w = 1.0
    grid.data = [0, 20, 40, 60, 80, 100, 120, -10, -100]

    # Publish the MarkerArray
    publisher.publish( grid )

    rospy.sleep(1)
