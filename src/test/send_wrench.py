#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
from geometry_msgs.msg import WrenchStamped
import math
import rospy

topic = 'test_wrench'
publisher = rospy.Publisher(topic, WrenchStamped)

rospy.init_node('send_wrench')

t = 0
while not rospy.is_shutdown():

   p = WrenchStamped()
   p.header.frame_id = "/base_link"
   p.header.stamp = rospy.Time.now()
   
   f = 0.5 * math.sin(t);
   p.wrench.force.x = 0
   p.wrench.force.y = 0
   p.wrench.force.z = f * math.sin( t )

   q = 0.5 + 0.5 * math.sin( t / 3.14 )
   p.wrench.torque.x = q * math.sin( t )
   p.wrench.torque.y = q * math.cos( t )
   p.wrench.torque.z = 0

   publisher.publish( p )

   t += .1

   rospy.sleep(0.03)
