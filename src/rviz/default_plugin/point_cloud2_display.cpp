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

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/time.h>

#include <rviz/default_plugin/point_cloud_common.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/int_property.h>
#include <rviz/validate_floats.h>

#include "point_cloud2_display.h"

namespace rviz
{
PointCloud2Display::PointCloud2Display() : point_cloud_common_(new PointCloudCommon(this))
{
  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.
  update_nh_.setCallbackQueue(point_cloud_common_->getCallbackQueue());
}

PointCloud2Display::~PointCloud2Display()
{
  delete point_cloud_common_;
}

void PointCloud2Display::onInitialize()
{
  MFDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
}

void PointCloud2Display::processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud)
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

  if (point_count * point_step != cloud->data.size())
  {
    std::stringstream ss;
    ss << "Data size (" << cloud->data.size() << " bytes) does not match width (" << cloud->width
       << ") times height (" << cloud->height << ") times point_step (" << point_step
       << ").  Dropping message.";
    setStatusStd(StatusProperty::Error, "Message", ss.str());
    return;
  }

  filtered->data.resize(cloud->data.size());
  uint32_t output_count;
  if (point_count == 0)
  {
    output_count = 0;
  }
  else
  {
    uint8_t* output_ptr = &filtered->data.front();
    const uint8_t *ptr = &cloud->data.front(), *ptr_end = &cloud->data.back(), *ptr_init;
    size_t points_to_copy = 0;
    for (; ptr < ptr_end; ptr += point_step)
    {
      float x = *reinterpret_cast<const float*>(ptr + xoff);
      float y = *reinterpret_cast<const float*>(ptr + yoff);
      float z = *reinterpret_cast<const float*>(ptr + zoff);
      if (validateFloats(x) && validateFloats(y) && validateFloats(z))
      {
        if (points_to_copy == 0)
        {
          // Only memorize where to start copying from
          ptr_init = ptr;
          points_to_copy = 1;
        }
        else
        {
          ++points_to_copy;
        }
      }
      else
      {
        if (points_to_copy)
        {
          // Copy all the points that need to be copied
          memcpy(output_ptr, ptr_init, point_step * points_to_copy);
          output_ptr += point_step * points_to_copy;
          points_to_copy = 0;
        }
      }
    }
    // Don't forget to flush what needs to be copied
    if (points_to_copy)
    {
      memcpy(output_ptr, ptr_init, point_step * points_to_copy);
      output_ptr += point_step * points_to_copy;
    }
    output_count = (output_ptr - &filtered->data.front()) / point_step;
  }

  filtered->header = cloud->header;
  filtered->fields = cloud->fields;
  filtered->data.resize(output_count * point_step);
  filtered->height = 1;
  filtered->width = output_count;
  filtered->is_bigendian = cloud->is_bigendian;
  filtered->point_step = point_step;
  filtered->row_step = output_count;

  point_cloud_common_->addMessage(filtered);
}


void PointCloud2Display::update(float wall_dt, float ros_dt)
{
  point_cloud_common_->update(wall_dt, ros_dt);
}

void PointCloud2Display::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::PointCloud2Display, rviz::Display)
