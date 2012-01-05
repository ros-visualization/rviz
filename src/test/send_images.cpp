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

#include "stdlib.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "math.h"
#include "image_transport/image_transport.h"

int main( int argc, char **argv )
{
  ros::init( argc, argv, "send_images" );

  ros::NodeHandle nh;
  image_transport::ImageTransport it( nh );
  image_transport::Publisher pub = it.advertise("images", 100);
  ros::Rate loop_rate( 100 );

  sensor_msgs::Image msg;
  int width = 100;
  int height = 1000;
  msg.data.resize( width * height * 3 );
  msg.header.frame_id = "base_link";
  msg.height = height;
  msg.width = width;
  msg.encoding = sensor_msgs::image_encodings::RGB8;
  msg.step = width * 3;

  int count = 0;
  while( ros::ok() )
  {
    for( int x = 0; x < width; x++ )
    {
      for( int y = 0; y < height; y++ )
      {
        int index = (x + y * width) * 3;
        long int rand = random();
        msg.data[ index ] = rand & 0xff;
        index++;
        msg.data[ index ] = (rand >> 8) & 0xff;
        index++;
        msg.data[ index ] = (rand >> 16) & 0xff;
      }
    }
    msg.header.seq = count;
    msg.header.stamp = ros::Time::now();

    pub.publish( msg );

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}
