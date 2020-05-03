#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import math
import rospy

topic = 'test_pose'
publisher = rospy.Publisher(topic, PoseStamped)

rospy.init_node('send_pose')

t = 0
while not rospy.is_shutdown():

   p = PoseStamped()
   p.header.frame_id = "base_link"
   p.header.stamp = rospy.Time.now()

   r = 5.0
   p.pose.position.x = r * math.cos( t )
   p.pose.position.y = r * math.sin( t )
   p.pose.position.z = 0
   p.pose.orientation.x = 0
   p.pose.orientation.y = 0
   p.pose.orientation.z = math.sin( .5 * t )
   p.pose.orientation.w = math.cos( .5 * t )

   publisher.publish( p )

   t += .1

   rospy.sleep(0.03)
