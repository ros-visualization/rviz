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

#include "laser_scan_display.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"
#include "ros_topic_property.h"

#include "ros/node.h"
#include "ogre_tools/point_cloud.h"

#include <tf/transform_listener.h>
#include <tf/message_notifier.h>
#include <robot_msgs/PointCloud.h>
#include <laser_scan/laser_scan.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

LaserScanDisplay::LaserScanDisplay( const std::string& name, VisualizationManager* manager )
: PointCloudBase( name, manager )
{
  projector_ = new laser_scan::LaserProjection();
  notifier_ = new tf::MessageNotifier<laser_scan::LaserScan>(tf_, ros_node_, boost::bind(&LaserScanDisplay::incomingScanCallback, this, _1), "", "", 10);
}

LaserScanDisplay::~LaserScanDisplay()
{
  delete notifier_;
  delete projector_;
}

void LaserScanDisplay::setTopic( const std::string& topic )
{
  topic_ = topic;

  if ( isEnabled() )
  {
    notifier_->setTopic( topic );
  }

  propertyChanged(topic_property_);

  causeRender();
}


void LaserScanDisplay::onEnable()
{
  PointCloudBase::onEnable();

  subscribe();
}

void LaserScanDisplay::onDisable()
{
  unsubscribe();

  PointCloudBase::onDisable();
}

void LaserScanDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  notifier_->setTopic( topic_ );
}

void LaserScanDisplay::unsubscribe()
{
  notifier_->setTopic( "" );
  notifier_->clear();
}


void LaserScanDisplay::incomingScanCallback(const boost::shared_ptr<laser_scan::LaserScan>& scan)
{
  boost::shared_ptr<robot_msgs::PointCloud> cloud(new robot_msgs::PointCloud);

  std::string frame_id = scan->header.frame_id;
  if ( frame_id.empty() )
  {
  	frame_id = fixed_frame_;
  }

  int mask = laser_scan::MASK_INTENSITY | laser_scan::MASK_DISTANCE | laser_scan::MASK_INDEX | laser_scan::MASK_TIMESTAMP;
  projector_->transformLaserScanToPointCloud(frame_id, *cloud, *scan , *tf_, mask);
  addMessage(cloud);
}

void LaserScanDisplay::targetFrameChanged()
{
}

void LaserScanDisplay::fixedFrameChanged()
{
  notifier_->setTargetFrame( fixed_frame_ );

  PointCloudBase::fixedFrameChanged();
}

void LaserScanDisplay::createProperties()
{
  PointCloudBase::createProperties();

  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &LaserScanDisplay::getTopic, this ),
                                                                            boost::bind( &LaserScanDisplay::setTopic, this, _1 ), category_, this );
  ROSTopicStringPropertyPtr str_prop = topic_property_.lock();
  str_prop->addLegacyName("Scan Topic");
  str_prop->setMessageType(laser_scan::LaserScan::__s_getDataType());
}

const char* LaserScanDisplay::getDescription()
{
  return "Displays the data from a laser_scan::LaserScan message, with the option to accumulate over a period of time.";
}

} // namespace rviz
