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
#include "rviz/frame_manager.h"

#include "rviz/ogre_helpers/point_cloud.h"

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

LaserScanDisplay::LaserScanDisplay()
  : PointCloudBase()
{
}

LaserScanDisplay::~LaserScanDisplay()
{
  unsubscribe();
  tf_filter_->clear();
  delete projector_;
  delete tf_filter_;
}

void LaserScanDisplay::onInitialize()
{
  PointCloudBase::onInitialize();
  tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*vis_manager_->getTFClient(), "", 10, threaded_nh_);
  projector_ = new laser_geometry::LaserProjection();

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&LaserScanDisplay::incomingScanCallback, this, _1));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

void LaserScanDisplay::setQueueSize( int size )
{
  if( size != (int) tf_filter_->getQueueSize() )
  {
    tf_filter_->setQueueSize( (uint32_t) size );
    propertyChanged( queue_size_property_ );
  }
}

int LaserScanDisplay::getQueueSize()
{
  return (int) tf_filter_->getQueueSize();
}

void LaserScanDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;
  reset();

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

void LaserScanDisplay::unsubscribe()
{
  sub_.unsubscribe();
  tf_filter_->clear();
}


void LaserScanDisplay::incomingScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  sensor_msgs::PointCloudPtr cloud(new sensor_msgs::PointCloud);

  std::string frame_id = scan->header.frame_id;

  // Compute tolerance necessary for this scan
  ros::Duration tolerance(scan->time_increment * scan->ranges.size());
  if (tolerance > filter_tolerance_)
  {
    filter_tolerance_ = tolerance;
    tf_filter_->setTolerance(filter_tolerance_);
  }

  try
  {
    projector_->transformLaserScanToPointCloud(fixed_frame_, *scan, *cloud , *vis_manager_->getTFClient(), laser_geometry::channel_option::Intensity);
  }
  catch (tf::TransformException& e)
  {
    ROS_DEBUG("LaserScan [%s]: failed to transform scan: %s.  This message should not repeat (tolerance should now be set on our tf::MessageFilter).", name_.c_str(), e.what());
    return;
  }
  addMessage(cloud);
}

void LaserScanDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame(fixed_frame_);

  PointCloudBase::fixedFrameChanged();
}

void LaserScanDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &LaserScanDisplay::getTopic, this ),
                                                                            boost::bind( &LaserScanDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "sensor_msgs::LaserScan topic to subscribe to.");
  ROSTopicStringPropertyPtr str_prop = topic_property_.lock();
  str_prop->addLegacyName("Scan Topic");
  str_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::LaserScan>());

  queue_size_property_ = property_manager_->createProperty<IntProperty>( "Queue Size", property_prefix_,
                                                                         boost::bind( &LaserScanDisplay::getQueueSize, this ),
                                                                         boost::bind( &LaserScanDisplay::setQueueSize, this, _1 ),
                                                                         parent_category_, this );
  setPropertyHelpText( queue_size_property_, "Advanced: set the size of the incoming LaserScan message queue.  Increasing this is useful if your incoming TF data is delayed significantly from your LaserScan data, but it can greatly increase memory usage if the messages are big." );

  PointCloudBase::createProperties();
}

} // namespace rviz
