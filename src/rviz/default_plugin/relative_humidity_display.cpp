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

#include "rviz/default_plugin/point_cloud_common.h"
#include "rviz/default_plugin/point_cloud_transformers.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/int_property.h"
#include "rviz/validate_floats.h"

#include "relative_humidity_display.h"

namespace rviz
{
RelativeHumidityDisplay::RelativeHumidityDisplay() : point_cloud_common_(new PointCloudCommon(this))
{
  queue_size_property_ =
      new IntProperty("Queue Size", 10,
                      "Advanced: set the size of the incoming RelativeHumidity message queue. "
                      " Increasing this is useful if your incoming TF data is delayed significantly "
                      "from your RelativeHumidity data, but it can greatly increase memory usage if the "
                      "messages are big.",
                      this, SLOT(updateQueueSize()));

  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.
  update_nh_.setCallbackQueue(point_cloud_common_->getCallbackQueue());
}

RelativeHumidityDisplay::~RelativeHumidityDisplay()
{
  delete point_cloud_common_;
}

void RelativeHumidityDisplay::onInitialize()
{
  MFDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);

  // Set correct initial values
  subProp("Channel Name")->setValue("relative_humidity");
  subProp("Autocompute Intensity Bounds")->setValue(false);
  subProp("Min Intensity")->setValue(0.0); // 0% relative humidity
  subProp("Max Intensity")->setValue(1.0); // 100% relative humidity
}

void RelativeHumidityDisplay::updateQueueSize()
{
  tf_filter_->setQueueSize((uint32_t)queue_size_property_->getInt());
}

void RelativeHumidityDisplay::processMessage(const sensor_msgs::RelativeHumidityConstPtr& msg)
{
  sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);

  // Create fields
  sensor_msgs::PointField x;
  x.name = "x";
  x.offset = 0;
  x.datatype = sensor_msgs::PointField::FLOAT32;
  x.count = 1;
  sensor_msgs::PointField y;
  y.name = "y";
  y.offset = 4;
  y.datatype = sensor_msgs::PointField::FLOAT32;
  y.count = 1;
  sensor_msgs::PointField z;
  z.name = "z";
  z.offset = 8;
  z.datatype = sensor_msgs::PointField::FLOAT32;
  z.count = 1;
  sensor_msgs::PointField relative_humidity;
  relative_humidity.name = "relative_humidity";
  relative_humidity.offset = 12;
  relative_humidity.datatype = sensor_msgs::PointField::FLOAT64;
  relative_humidity.count = 1;

  // Create pointcloud from message
  filtered->header = msg->header;
  filtered->fields.push_back(x);
  filtered->fields.push_back(y);
  filtered->fields.push_back(z);
  filtered->fields.push_back(relative_humidity);
  filtered->data.resize(20);
  const float zero_float = 0.0; // RelativeHumidity is always on its tf frame
  memcpy(&filtered->data[x.offset], &zero_float, 4);
  memcpy(&filtered->data[y.offset], &zero_float, 4);
  memcpy(&filtered->data[z.offset], &zero_float, 4);
  memcpy(&filtered->data[relative_humidity.offset], &msg->relative_humidity, 8);
  filtered->height = 1;
  filtered->width = 1;
  filtered->is_bigendian = false;
  filtered->point_step = 20;
  filtered->row_step = 1;

  // Give to point_cloud_common to draw
  point_cloud_common_->addMessage(filtered);
}


void RelativeHumidityDisplay::update(float wall_dt, float ros_dt)
{
  point_cloud_common_->update(wall_dt, ros_dt);

  // Hide unneeded properties
  subProp("Position Transformer")->hide();
  subProp("Color Transformer")->hide();
  subProp("Channel Name")->hide();
  subProp("Autocompute Intensity Bounds")->hide();
}

void RelativeHumidityDisplay::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::RelativeHumidityDisplay, rviz::Display)
