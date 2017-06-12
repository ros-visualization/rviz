/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <stdio.h>

#include <gtest/gtest.h>

#include <rviz/properties/property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/enum_property.h>

#include "mock_property_change_receiver.h"

using namespace rviz;

TEST( QueueSizeProperty, create_and_get )
{
  IntProperty* queue_size_property_ = new IntProperty( "Queue Size", 10,
                                          "Size of the tf message filter queue. It usually needs to be set at least as high as the number of sonar frames.");
  int queue_length_ = queue_size_property_->getInt();
  EXPECT_EQ( 10, queue_length_ );
}

TEST( QueueSizeProperty, set_and_get )
{
  IntProperty* queue_size_property_ = new IntProperty( "Queue Size", 10,
                                          "Size of the tf message filter queue. It usually needs to be set at least as high as the number of sonar frames.");
  queue_size_property_->setInt(20);
  int queue_length_ = queue_size_property_->getInt();
  EXPECT_EQ( 20, queue_length_ );
}

int main( int argc, char **argv ) {
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
