#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
import math
import rospy

topic = 'test_polygon'
publisher = rospy.Publisher(topic, PolygonStamped)

rospy.init_node('send_polygon')

t = 0
while not rospy.is_shutdown():

   p = PolygonStamped()
   p.header.frame_id = "/base_link"
   p.header.stamp = rospy.Time.now()
   
   dr = 0.5 * math.cos( t )
   radii = [ 1-dr, 1+dr ]
   radius_index = 0
   num_points = 10
   for i in range( 0, num_points ):
      point = Point32()
      radius = radii[ radius_index ]
      radius_index = (radius_index + 1) % 2
      point.x = radius * math.cos( i * 2 * math.pi / num_points )
      point.y = radius * math.sin( i * 2 * math.pi / num_points )
      point.z = 0
      p.polygon.points.append( point )

   publisher.publish( p )

   t += .1

   rospy.sleep(0.03)
