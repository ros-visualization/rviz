#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
import math

import tf

if __name__ == "__main__":
    rospy.init_node("send_tf")

    br = tf.TransformBroadcaster()
    r = rospy.Rate(10)
    t = 0
    while not rospy.is_shutdown():
        for m in range(1, 20):
            br.sendTransform(
                (0, 0, 0),
                tf.transformations.quaternion_from_euler(0, 0.8 * m, 0),
                rospy.Time.now(),
                "frame0_{0}".format(m),
                "base_link",
            )
            for n in range(1, 20):
                br.sendTransform(
                    (0.2 * math.sin(t * 10 + n), 0, 0.2),
                    tf.transformations.quaternion_from_euler(0, 0.6 * math.sin(t), 0.2),
                    rospy.Time.now(),
                    "frame{0}_{1}".format(n, m),
                    "frame{0}_{1}".format(n - 1, m),
                )
        t += 0.01
        r.sleep()
