#!/usr/bin/env python

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import rospy

topic = 'test_poses'
publisher = rospy.Publisher(topic, PoseArray)

rospy.init_node('posearray')

while not rospy.is_shutdown():

   ps = PoseArray()
   ps.header.frame_id = "/base_link"
   ps.header.stamp = rospy.Time.now()

   pose = Pose()
   pose.position.x = 2
   pose.position.y = 2
   pose.position.z = 0
   pose.orientation.x = 0
   pose.orientation.y = 0
   pose.orientation.z = .7071
   pose.orientation.w = .7071

   ps.poses.append( pose )

   pose = Pose()
   pose.position.x = 1
   pose.position.y = 1
   pose.position.z = 0
   pose.orientation.x = 0
   pose.orientation.y = 0
   pose.orientation.z = 0
   pose.orientation.w = 1

   ps.poses.append( pose )

   publisher.publish( ps )

   rospy.sleep(0.1)
