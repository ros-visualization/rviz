#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
#!/usr/bin/env python

import rospy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

pub1 = rospy.Publisher("cloud1", PointCloud)
pub2 = rospy.Publisher("cloud2", PointCloud)

rospy.init_node('send_two_clouds')

cloud = PointCloud()
while not rospy.is_shutdown():

    cloud.header.frame_id = "/base_link"
    cloud.header.stamp = rospy.Time.now()

    cloud.points = [
        Point32( 0, 0, 0 ),
        Point32( .1, 0, 0 ),
        Point32( 0, .1, 0 ),
        Point32( .1, .1, 0 )
    ]
    pub1.publish( cloud )

    cloud.points = [
        Point32( .5, 0, 0 ),
        Point32( .6, 0, 0 ),
        Point32( .5, .1, 0 ),
        Point32( .6, .1, 0 )
    ]
    pub2.publish( cloud )

    rospy.sleep(.5)
