#!/usr/bin/env python

from sensor_msgs.msg import JointState
from math import pi, sin
import rospy

topic = 'joint_states'
publisher = rospy.Publisher(topic, JointState, queue_size=1)
rospy.init_node('send_efforts')

t = 0
msg = JointState()
msg.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
'panda_joint6', 'panda_joint7', 'panda_finger_joint1']
N = len(msg.name)
msg.position = [0. for i in range(N)]

while not rospy.is_shutdown():
	msg.header.stamp = rospy.Time.now()
	msg.effort = [10. * sin(t) for i in range(N)]
	publisher.publish(msg)
	t = t + 0.1
	rospy.sleep(0.1)
