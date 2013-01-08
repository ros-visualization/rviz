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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ros/time.h>

#include "rviz/default_plugin/point_cloud_common.h"
#include "rviz/default_plugin/point_cloud_transformers.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/int_property.h"
#include "rviz/validate_floats.h"

#include "point_cloud2_display.h"

namespace rviz
{

PointCloud2Display::PointCloud2Display()
  : point_cloud_common_( new PointCloudCommon( this ))
{
  queue_size_property_ = new IntProperty( "Queue Size", 10,
                                          "Advanced: set the size of the incoming PointCloud2 message queue. "
                                          " Increasing this is useful if your incoming TF data is delayed significantly "
                                          "from your PointCloud2 data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));

  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.
  update_nh_.setCallbackQueue( point_cloud_common_->getCallbackQueue() );
}

PointCloud2Display::~PointCloud2Display()
{
  delete point_cloud_common_;
}

void PointCloud2Display::onInitialize()
{
  MFDClass::onInitialize();
  point_cloud_common_->initialize( context_, scene_node_ );
}

void PointCloud2Display::updateQueueSize()
{
  tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
}

void PointCloud2Display::processMessage( const sensor_msgs::PointCloud2ConstPtr& cloud )
{
  // Filter any nan values out of the cloud.  Any nan values that make it through to PointCloudBase
  // will get their points put off in lala land, but it means they still do get processed/rendered
  // which can be a big performance hit
  sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);
  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  if (xi == -1 || yi == -1 || zi == -1)
  {
    return;
  }

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint32_t point_step = cloud->point_step;
  const size_t point_count = cloud->width * cloud->height;
  filtered->data.resize(cloud->data.size());
  if (point_count == 0)
  {
    return;
  }

  uint32_t output_count = 0;
  const uint8_t* ptr = &cloud->data.front();
  for (size_t i = 0; i < point_count; ++i)
  {
    float x = *reinterpret_cast<const float*>(ptr + xoff);
    float y = *reinterpret_cast<const float*>(ptr + yoff);
    float z = *reinterpret_cast<const float*>(ptr + zoff);
    if (validateFloats(Ogre::Vector3(x, y, z)))
    {
      memcpy(&filtered->data.front() + (output_count * point_step), ptr, point_step);
      ++output_count;
    }

    ptr += point_step;
  }

  filtered->header = cloud->header;
  filtered->fields = cloud->fields;
  filtered->data.resize(output_count * point_step);
  filtered->height = 1;
  filtered->width = output_count;
  filtered->is_bigendian = cloud->is_bigendian;
  filtered->point_step = point_step;
  filtered->row_step = output_count;

  point_cloud_common_->addMessage( filtered );
}


void PointCloud2Display::update( float wall_dt, float ros_dt )
{
  point_cloud_common_->update( wall_dt, ros_dt );
}

void PointCloud2Display::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::PointCloud2Display, rviz::Display )
