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
#include "common.h"
#include "ros_topic_property.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include "ros/node.h"
#include <ros/time.h>
#include "ogre_tools/point_cloud.h"

#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace rviz
{

PointCloudDisplay::PointCloudDisplay( const std::string& name, VisualizationManager* manager )
: PointCloudBase( name, manager )
, topic_property_( NULL )
{
  notifier_ = new tf::MessageNotifier<robot_msgs::PointCloud>(tf_, ros_node_, boost::bind(&PointCloudDisplay::incomingCloudCallback, this, _1), "", "", 10);
}

PointCloudDisplay::~PointCloudDisplay()
{
  unsubscribe();

  delete notifier_;
}

void PointCloudDisplay::setTopic( const std::string& topic )
{
  topic_ = topic;

  if ( isEnabled() )
  {
    notifier_->setTopic( topic );
  }

  if ( topic_property_ )
  {
    topic_property_->changed();
  }

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
  notifier_->clear();

  PointCloudBase::onDisable();
}

void PointCloudDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  notifier_->setTopic( topic_ );
}

void PointCloudDisplay::unsubscribe()
{
  notifier_->setTopic( "" );
}

void PointCloudDisplay::incomingCloudCallback(const boost::shared_ptr<robot_msgs::PointCloud>& cloud)
{
  addMessage(cloud);
}

void PointCloudDisplay::targetFrameChanged()
{
}

void PointCloudDisplay::fixedFrameChanged()
{
  notifier_->setTargetFrame( fixed_frame_ );
}

void PointCloudDisplay::createProperties()
{
  PointCloudBase::createProperties();

  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PointCloudDisplay::getTopic, this ),
                                                                              boost::bind( &PointCloudDisplay::setTopic, this, _1 ), parent_category_, this );
  topic_property_->setMessageType(robot_msgs::PointCloud::__s_getDataType());

}

const char* PointCloudDisplay::getDescription()
{
  return "Displays a point cloud from a robot_msgs::PointCloud message, with the option to accumulate over time.";
}

} // namespace rviz
