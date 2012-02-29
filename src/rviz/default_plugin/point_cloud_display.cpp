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
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"

#include <ros/time.h>
#include "rviz/ogre_helpers/point_cloud.h"

#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

PointCloudDisplay::PointCloudDisplay()
  : PointCloudBase()
  , tf_filter_( 0 )
{
}

PointCloudDisplay::~PointCloudDisplay()
{
  unsubscribe();
  tf_filter_->clear();
  delete tf_filter_;
}

void PointCloudDisplay::onInitialize()
{
  PointCloudBase::onInitialize();
  tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>( *vis_manager_->getTFClient(), "", 10, threaded_nh_ );
  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(&PointCloudDisplay::incomingCloudCallback, this);
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

void PointCloudDisplay::setQueueSize( int size )
{
  if( size != (int) tf_filter_->getQueueSize() )
  {
    tf_filter_->setQueueSize( (uint32_t) size );
    propertyChanged( queue_size_property_ );
  }
}

int PointCloudDisplay::getQueueSize()
{
  return (int) tf_filter_->getQueueSize();
}

void PointCloudDisplay::setTopic( const std::string& topic )
{
  unsubscribe();
  topic_ = topic;
  reset();
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
  tf_filter_->clear();

  PointCloudBase::onDisable();
}

void PointCloudDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe(threaded_nh_, topic_, 2);
    setStatus(status_levels::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
  }
}

void PointCloudDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void PointCloudDisplay::incomingCloudCallback(const sensor_msgs::PointCloudConstPtr& cloud)
{
  addMessage(cloud);
}

void PointCloudDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_ );

  PointCloudBase::fixedFrameChanged();
}

void PointCloudDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PointCloudDisplay::getTopic, this ),
                                                                              boost::bind( &PointCloudDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "sensor_msgs::PointCloud topic to subscribe to.");
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::PointCloud>());

  queue_size_property_ = property_manager_->createProperty<IntProperty>( "Queue Size", property_prefix_,
                                                                         boost::bind( &PointCloudDisplay::getQueueSize, this ),
                                                                         boost::bind( &PointCloudDisplay::setQueueSize, this, _1 ),
                                                                         parent_category_, this );
  setPropertyHelpText( queue_size_property_, "Advanced: set the size of the incoming PointCloud message queue.  Increasing this is useful if your incoming TF data is delayed significantly from your PointCloud data, but it can greatly increase memory usage if the messages are big." );

  PointCloudBase::createProperties();
}

} // namespace rviz
