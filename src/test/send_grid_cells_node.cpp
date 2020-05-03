/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "nav_msgs/GridCells.h"
#include "math.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_grid_cells");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<nav_msgs::GridCells>("grid_cells", 100);
  ros::Rate loop_rate(100);

  nav_msgs::GridCells msg;
  int width = 500;
  int length = 500;
  msg.cells.resize(width * length);
  msg.header.frame_id = "base_link";
  msg.cell_width = .01;
  msg.cell_height = .01;

  int count = 0;
  while (ros::ok())
  {
    for (int x = 0; x < width; x++)
    {
      for (int y = 0; y < length; y++)
      {
        geometry_msgs::Point& point = msg.cells[x + y * width];
        point.x = x / 100.0;
        point.y = y / 100.0;
        point.z = sin(x / 100.0 + y / 100.0 + count / 100.0);
        //        point.z = ((x + y + count) % 100) / 100.0;
      }
    }
    msg.header.seq = count;
    msg.header.stamp = ros::Time::now();

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}
