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
#include <gtest/gtest.h>

#include <QVariant>

#include "rviz/properties/topic_info_variant.h"

TEST( RosTopicVar, basic )
{
  ros::master::TopicInfo rtv;
  rtv.datatype = "std_msgs/String";
  rtv.name = "/ns/chatter";

  QVariant var = 7;
  EXPECT_EQ( var.type(), QVariant::Int );
  EXPECT_FALSE( var.canConvert<ros::master::TopicInfo>() );
  
  var = QVariant::fromValue( rtv );

// For some reason these are not equal!  Looks like QVariant has a
// different idea about typeIds than qMetaTypeId does.  Result: don't
// use qMetaTypeId<>() to identify QVariant types, just use
// QVariant::canConvert<>().
//
//  EXPECT_EQ( var.type(), qMetaTypeId<ros::master::TopicInfo>() );

  EXPECT_TRUE( var.canConvert<ros::master::TopicInfo>() );

  QVariant var2 = var;

  ros::master::TopicInfo rtv2 = var.value<ros::master::TopicInfo>();
  EXPECT_EQ( "std_msgs/String", rtv2.datatype );
  EXPECT_EQ( "/ns/chatter", rtv2.name );

  ros::master::TopicInfo rtv3 = var2.value<ros::master::TopicInfo>();
  EXPECT_EQ( "std_msgs/String", rtv3.datatype );
  EXPECT_EQ( "/ns/chatter", rtv3.name );
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
