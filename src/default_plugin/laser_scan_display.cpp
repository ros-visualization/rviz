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
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/common.h"

#include "ogre_tools/point_cloud.h"

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

LaserScanDisplay::LaserScanDisplay( const std::string& name, VisualizationManager* manager )
: PointCloudBase( name, manager )
, tf_filter_(*manager->getThreadedTFClient(), "", 10, threaded_nh_)
{
  projector_ = new laser_geometry::LaserProjection();

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&LaserScanDisplay::incomingScanCallback, this, _1));
}

LaserScanDisplay::~LaserScanDisplay()
{
  delete projector_;
}

void LaserScanDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

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

  sub_.subscribe(threaded_nh_, topic_, 10);
}

void LaserScanDisplay::unsubscribe()
{
  sub_.unsubscribe();
  tf_filter_.clear();
}


void LaserScanDisplay::incomingScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  sensor_msgs::PointCloud::Ptr cloud(new sensor_msgs::PointCloud);

  std::string frame_id = scan->header.frame_id;
  if ( frame_id.empty() )
  {
  	frame_id = fixed_frame_;
  }

  int mask = laser_geometry::MASK_INTENSITY | laser_geometry::MASK_DISTANCE | laser_geometry::MASK_INDEX | laser_geometry::MASK_TIMESTAMP;
  projector_->transformLaserScanToPointCloud(frame_id, *cloud, *scan , *vis_manager_->getThreadedTFClient(), mask);
  addMessage(cloud);
}

void LaserScanDisplay::targetFrameChanged()
{
}

void LaserScanDisplay::fixedFrameChanged()
{
  tf_filter_.setTargetFrame(fixed_frame_);

  PointCloudBase::fixedFrameChanged();
}

void LaserScanDisplay::createProperties()
{
  PointCloudBase::createProperties();

  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &LaserScanDisplay::getTopic, this ),
                                                                            boost::bind( &LaserScanDisplay::setTopic, this, _1 ), parent_category_, this );
  ROSTopicStringPropertyPtr str_prop = topic_property_.lock();
  str_prop->addLegacyName("Scan Topic");
  str_prop->setMessageType(sensor_msgs::LaserScan::__s_getDataType());
}

} // namespace rviz
