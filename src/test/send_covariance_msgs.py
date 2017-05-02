#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Ellon Paiva Mendes @ LAAS-CNRS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest( 'rviz_plugin_covariance' )
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import rospy
from math import cos, sin, pi
import tf
import tf_conversions

publisher_cov = rospy.Publisher( 'pose_with_cov', PoseWithCovarianceStamped, queue_size=5 )
publisher_pose = rospy.Publisher( 'pose', PoseStamped, queue_size=5 )

rospy.init_node( 'test_covariance' )

br = tf.TransformBroadcaster()
rate = rospy.Rate(100)
# radius = 1
angle = 0
# r = 0
# p = 0
# y = 0

linear_deviation = 0.5;


while not rospy.is_shutdown():
    stamp = rospy.Time.now()

    # Define static pose with covariance
    pose_with_cov = PoseWithCovarianceStamped()
    pose_with_cov.header.frame_id = "/base_link"
    pose_with_cov.header.stamp = stamp

    pose_with_cov.pose.pose.position.x = 3
    pose_with_cov.pose.pose.position.y = 3
    pose_with_cov.pose.pose.position.z = 3

    ori = pose_with_cov.pose.pose.orientation
    ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(pi/2,pi/3,0)

    pose_with_cov.pose.covariance[0] = linear_deviation**2.0
    pose_with_cov.pose.covariance[6+1] = 0.0001
    pose_with_cov.pose.covariance[12+2] = 0.0001
    pose_with_cov.pose.covariance[18+3] = 0.01
    pose_with_cov.pose.covariance[24+4] = 0.01
    pose_with_cov.pose.covariance[30+5] = 0.01

    # Define a dynamic pose that should move inside the deviation
    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = stamp

    pose.pose.position.x = pose_with_cov.pose.pose.position.x + linear_deviation*cos( 10 * angle )
    pose.pose.position.y = pose_with_cov.pose.pose.position.y
    pose.pose.position.z = pose_with_cov.pose.pose.position.z

    ori = pose.pose.orientation
    ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(pi/2,pi/3,0)

    publisher_cov.publish( pose_with_cov )
    publisher_pose.publish( pose )

    # br.sendTransform((radius * cos(angle), radius * sin(angle), 0),
    #                  tf.transformations.quaternion_from_euler(r, p, y),
    #                  rospy.Time.now(),
    #                  "base_link",
    #                  "map")
    pos, ori = pose_with_cov.pose.pose.position, pose_with_cov.pose.pose.orientation
    br.sendTransform((pos.x, pos.y, pos.z),
                     (ori.x, ori.y, ori.z, ori.w),
                     stamp,
                     "pose",
                     "base_link")

    angle += .0005
    # r = angle
    # p = angle
    # y = angle
    rate.sleep()

