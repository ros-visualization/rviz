#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import rospy

topic = 'test_path'
publisher = rospy.Publisher(topic, Path)

rospy.init_node('send_path')

t = 0
while not rospy.is_shutdown():

   p = Path()
   p.header.frame_id = "/base_link"
   p.header.stamp = rospy.Time.now()
   
   num_points = 100
   for i in range( 0, num_points ):
      ps = PoseStamped()
      ps.header.stamp = p.header.stamp
      ps.header.frame_id = p.header.frame_id
      ps.pose.position.x = 10.0 * i / num_points - 5
      ps.pose.position.y = math.sin( 10.0 * i / num_points + t )
      ps.pose.position.z = 0
      ps.pose.orientation.x = 0
      ps.pose.orientation.y = 0
      ps.pose.orientation.z = 0
      ps.pose.orientation.w = 1
      p.poses.append( ps )

   publisher.publish( p )

   t += .1

   rospy.sleep(0.03)
