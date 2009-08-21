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

#include "point_cloud_display.h"
#include "rviz/common.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"

#include <ros/time.h>
#include "ogre_tools/point_cloud.h"

#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

PointCloudDisplay::PointCloudDisplay( const std::string& name, VisualizationManager* manager )
: PointCloudBase( name, manager )
, tf_filter_(*manager->getThreadedTFClient(), "", 10, threaded_nh_)
{
  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&PointCloudDisplay::incomingCloudCallback, this, _1));
}

PointCloudDisplay::~PointCloudDisplay()
{
  unsubscribe();
}

void PointCloudDisplay::setTopic( const std::string& topic )
{
  unsubscribe();
  topic_ = topic;
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void PointCloudDisplay::onEnable()
{
  PointCloudBase::onEnable();

  subscribe();
}

void PointCloudDisplay::onDisable()
{
  unsubscribe();
  tf_filter_.clear();

  PointCloudBase::onDisable();
}

void PointCloudDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(threaded_nh_, topic_, 10);
}

void PointCloudDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void PointCloudDisplay::incomingCloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud)
{
  addMessage(cloud);
}

void PointCloudDisplay::targetFrameChanged()
{
}

void PointCloudDisplay::fixedFrameChanged()
{
  tf_filter_.setTargetFrame( fixed_frame_ );

  PointCloudBase::fixedFrameChanged();
}

void PointCloudDisplay::createProperties()
{
  PointCloudBase::createProperties();

  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PointCloudDisplay::getTopic, this ),
                                                                              boost::bind( &PointCloudDisplay::setTopic, this, _1 ), parent_category_, this );
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(sensor_msgs::PointCloud::__s_getDataType());

}

} // namespace rviz
