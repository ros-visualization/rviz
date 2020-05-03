#!/usr/bin/env python

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import rospy
import tf.transformations as tf

topic = 'test_path'
publisher = rospy.Publisher(topic, Path, queue_size=1)

rospy.init_node('send_path')

t = 0
rate = rospy.Rate(30)
timestep = rate.sleep_dur.to_sec()
while not rospy.is_shutdown():

   p = Path()
   p.header.frame_id = "/base_link"
   p.header.stamp = rospy.Time.now()

   num_points = 50
   for i in range( 0, num_points ):
      ps = PoseStamped()
      ps.header.stamp = p.header.stamp
      ps.header.frame_id = p.header.frame_id
      ps.pose.position.x = 10.0 * i / (num_points-1) - 5
      ps.pose.position.y = math.sin( 10.0 * i / num_points + t )
      ps.pose.position.z = 0
      angle = 2 * math.pi * i / num_points + t
      quat = tf.quaternion_from_euler(angle, 0, 0)
      ps.pose.orientation.x = quat[0]
      ps.pose.orientation.y = quat[1]
      ps.pose.orientation.z = quat[2]
      ps.pose.orientation.w = quat[3]
      p.poses.append( ps )

   publisher.publish( p )

   t += timestep
   rate.sleep()
