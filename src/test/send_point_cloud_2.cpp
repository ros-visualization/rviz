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
#include "sensor_msgs/PointCloud2.h"
#include "math.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_points2");

  int rate = 1;
  bool moving = true;
  int size = 100;

  if (argc > 1)
  {
    rate = atoi(argv[1]);
  }
  if (argc > 2)
  {
    moving = bool(atoi(argv[2]));
  }
  if (argc > 3)
  {
    size = atoi(argv[3]);
  }

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("points2", 10);
  ros::Rate loop_rate(rate);

  sensor_msgs::PointCloud2 msg;
  int width = size;
  int height = 2 * size;
  msg.header.frame_id = "base_link";
  msg.is_dense = false;
  msg.is_bigendian = false;

  msg.fields.resize(5);

  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;

  msg.fields[3].name = "rgb";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::PointField::INT32;
  msg.fields[3].count = 1;

  msg.fields[4].name = "joy";
  msg.fields[4].offset = 16;
  msg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[4].count = 1;

  msg.point_step = 20;

  int count = 0;
  while (ros::ok())
  {
    width++;
    int num_points = width * height;
    msg.row_step = width * msg.point_step;
    msg.height = height;
    msg.width = width;
    msg.data.resize(num_points * msg.point_step);
    for (int x = 0; x < width; x++)
    {
      for (int y = 0; y < height; y++)
      {
        uint8_t* ptr = &msg.data[0] + (x + y * width) * msg.point_step;
        *(float*)ptr = x / 100.0f;
        ptr += 4;
        *(float*)ptr = y / 100.0f;
        ptr += 4;
        *(float*)ptr = 0.1 * sinf(x / 10.0f) * sinf(y / 10.0f);
        ptr += 4;
        *ptr = (x + count) & 0xff;
        ptr++;
        *ptr = y & 0xff;
        ptr++;
        *ptr = (x + y) & 0xff;
        ptr++;
        ptr++;
        *(float*)ptr = 127.0f + 127.0f * sinf((x - count) / 10.0f) * sinf(y / 10.0f);
        // ptr += 4;
      }
    }
    msg.header.seq = count;
    msg.header.stamp = ros::Time::now();

    printf("publishing at %d hz, %s, %d x %d points.\n", rate, (moving ? "moving" : "static"), width,
           height);

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    if (moving)
    {
      ++count;
    }
  }
}
