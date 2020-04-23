#!/usr/bin/env python  
# Test script to publish a "cycling_frame" that rotates about the origin

import math
import rospy
import tf2_ros
import geometry_msgs.msg

rospy.init_node('tf2_cycling_frame_broadcaster')
broadcaster = tf2_ros.TransformBroadcaster()
rate = rospy.Rate(30)

msg = geometry_msgs.msg.TransformStamped()
msg.header.frame_id = "base_link"
msg.child_frame_id = "cycling_frame"
t = 0

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    msg.transform.translation.x = math.cos(t)
    msg.transform.translation.y = math.sin(t)
    msg.transform.rotation.w = math.cos(t/2)
    msg.transform.rotation.z = math.sin(t/2)
    broadcaster.sendTransform(msg)
    t += 0.01
    rate.sleep()
