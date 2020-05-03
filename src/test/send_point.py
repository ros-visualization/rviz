#!/usr/bin/env python

from geometry_msgs.msg import PointStamped
import math
import rospy

topic = 'test_point'
publisher = rospy.Publisher(topic, PointStamped)

rospy.init_node('send_point')

t = 0
while not rospy.is_shutdown():

   p = PointStamped()
   p.header.frame_id = "base_link"
   p.header.stamp = rospy.Time.now()

   r = 5.0
   p.point.x = r * math.cos( t )
   p.point.y = r * math.sin( t )
   p.point.z = 0

   publisher.publish( p )

   t += .1

   rospy.sleep(0.03)
