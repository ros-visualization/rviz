/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "point_cloud2_display.h"
#include "point_cloud_transformers.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include <ros/time.h>
#include "ogre_tools/point_cloud.h"

#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

PointCloud2Display::PointCloud2Display( const std::string& name, VisualizationManager* manager )
: PointCloudBase( name, manager )
, tf_filter_(*manager->getTFClient(), "", 10, threaded_nh_)
{
  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(&PointCloud2Display::incomingCloudCallback, this);
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

PointCloud2Display::~PointCloud2Display()
{
  unsubscribe();
  tf_filter_.clear();
}

void PointCloud2Display::setTopic( const std::string& topic )
{
  unsubscribe();
  topic_ = topic;
  reset();
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void PointCloud2Display::onEnable()
{
  PointCloudBase::onEnable();

  subscribe();
}

void PointCloud2Display::onDisable()
{
  unsubscribe();
  tf_filter_.clear();

  PointCloudBase::onDisable();
}

void PointCloud2Display::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(threaded_nh_, topic_, 2);
}

void PointCloud2Display::unsubscribe()
{
  sub_.unsubscribe();
}

void PointCloud2Display::incomingCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
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

  addMessage(filtered);
}

void PointCloud2Display::targetFrameChanged()
{
}

void PointCloud2Display::fixedFrameChanged()
{
  tf_filter_.setTargetFrame( fixed_frame_ );

  PointCloudBase::fixedFrameChanged();
}

void PointCloud2Display::createProperties()
{
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PointCloud2Display::getTopic, this ),
                                                                              boost::bind( &PointCloud2Display::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "sensor_msgs::PointCloud2 topic to subscribe to.");
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::PointCloud2>());

  PointCloudBase::createProperties();
}

} // namespace rviz
