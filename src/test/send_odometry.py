#!/usr/bin/env python

from nav_msgs.msg import Odometry
import rospy

topic = 'test_odometry'
publisher = rospy.Publisher(topic, Odometry)

rospy.init_node('send_odometry')

y = 0
while not rospy.is_shutdown():

   odo = Odometry()
   odo.header.frame_id = "/base_link"
   odo.header.stamp = rospy.Time.now()

   odo.pose.pose.position.x = 0
   odo.pose.pose.position.y = y
   odo.pose.pose.position.z = 0

   odo.pose.pose.orientation.x = 0
   odo.pose.pose.orientation.y = 0
   odo.pose.pose.orientation.z = 0
   odo.pose.pose.orientation.w = 1

   publisher.publish( odo )

   y = y + .2
   if y > 5:
      y = -5

   rospy.sleep(0.1)
