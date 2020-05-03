#!/usr/bin/env python

from sensor_msgs.msg import Range
import rospy

topic = 'test_range'
publisher = rospy.Publisher(topic, Range)

rospy.init_node('range_test')

dist = 3

while not rospy.is_shutdown():

   r = Range()
   r.header.frame_id = "base_link"
   r.header.stamp = rospy.Time.now()

   r.radiation_type = 0
   r.field_of_view = 2.0/dist
   r.min_range = .4
   r.max_range = 10
   r.range = dist

   publisher.publish(r)

   rospy.sleep(0.1)

   dist += .3
   if dist > 10:
      dist = 3
