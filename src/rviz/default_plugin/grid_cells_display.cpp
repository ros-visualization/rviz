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

#include "grid_cells_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include "rviz/ogre_helpers/arrow.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <rviz/ogre_helpers/point_cloud.h>

namespace rviz
{

GridCellsDisplay::GridCellsDisplay()
  : Display()
  , color_( 0.1f, 1.0f, 0.0f )
  , messages_received_(0)
  , last_frame_count_( uint64_t( -1 ))
  , hidden_(false)
{
}

void GridCellsDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<nav_msgs::GridCells>(*vis_manager_->getTFClient(), "", 10, update_nh_);
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "PolyLine" << count++;

  cloud_ = new PointCloud();
  cloud_->setRenderMode( PointCloud::RM_BILLBOARDS_COMMON_FACING );
  cloud_->setCommonDirection( Ogre::Vector3::UNIT_Z );
  cloud_->setCommonUpVector( Ogre::Vector3::UNIT_Y );
  scene_node_->attachObject(cloud_);
  setAlpha( 1.0f );

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&GridCellsDisplay::incomingMessage, this, _1));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

GridCellsDisplay::~GridCellsDisplay()
{
  unsubscribe();
  clear();

  scene_manager_->destroySceneNode(scene_node_->getName());
  delete cloud_;
  delete tf_filter_;
}

void GridCellsDisplay::clear()
{
  cloud_->clear();

  messages_received_ = 0;
  setStatus(status_levels::Warn, "Topic", "No messages received");
}

void GridCellsDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void GridCellsDisplay::setColor( const Color& color )
{
  color_ = color;

  propertyChanged(color_property_);

  processMessage(current_message_);
  causeRender();
}

void GridCellsDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  cloud_->setAlpha( alpha );

  propertyChanged(alpha_property_);

  processMessage(current_message_);
  causeRender();
}

void GridCellsDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe(update_nh_, topic_, 10);
    setStatus(status_levels::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
  }
}

void GridCellsDisplay::unsubscribe()
{
  sub_.unsubscribe();
}


void GridCellsDisplay::onEnable()
{
  subscribe();
  scene_node_->setVisible( enabled_ && !hidden_ );
}

void GridCellsDisplay::onDisable()
{
  unsubscribe();
  scene_node_->setVisible( enabled_ && !hidden_ );
  clear();
}

void GridCellsDisplay::hideVisible()
{
  hidden_ = true;
  scene_node_->setVisible( enabled_ && !hidden_ );
}

void GridCellsDisplay::restoreVisible()
{
  hidden_ = false;
  scene_node_->setVisible( enabled_ && !hidden_ );
}

void GridCellsDisplay::fixedFrameChanged()
{
  clear();

  tf_filter_->setTargetFrame( fixed_frame_ );
}

void GridCellsDisplay::update(float wall_dt, float ros_dt)
{
}

bool validateFloats(const nav_msgs::GridCells& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.cell_width);
  valid = valid && validateFloats(msg.cell_height);
  valid = valid && validateFloats(msg.cells);
  return valid;
}

void GridCellsDisplay::processMessage(const nav_msgs::GridCells::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  ++messages_received_;

  if( vis_manager_->getFrameCount() == last_frame_count_ )
  {
    return;
  }
  last_frame_count_ = vis_manager_->getFrameCount();

  cloud_->clear();

  if (!validateFloats(*msg))
  {
    setStatus(status_levels::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
    return;
  }

  std::stringstream ss;
  ss << messages_received_ << " messages received";
  setStatus(status_levels::Ok, "Topic", ss.str());

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!vis_manager_->getFrameManager()->getTransform(msg->header, position, orientation))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  Ogre::ColourValue color( color_.r_, color_.g_, color_.b_, alpha_ );

  if( msg->cell_width == 0 )
  {
    setStatus(status_levels::Error, "Topic", "Cell width is zero, cells will be invisible.");
  }
  else if( msg->cell_height == 0 )
  {
    setStatus(status_levels::Error, "Topic", "Cell height is zero, cells will be invisible.");
  }

  cloud_->setDimensions(msg->cell_width, msg->cell_height, 0.0);

  uint32_t num_points = msg->cells.size();

  typedef std::vector< PointCloud::Point > V_Point;
  V_Point points;
  points.resize( num_points );
  for(uint32_t i = 0; i < num_points; i++)
  {
    PointCloud::Point& current_point = points[ i ];

    Ogre::Vector3 pos(msg->cells[i].x, msg->cells[i].y, msg->cells[i].z);

    current_point.x = pos.x;
    current_point.y = pos.y;
    current_point.z = pos.z;
    current_point.setColor(color.r, color.g, color.b);
  }

  cloud_->clear();

  if ( !points.empty() )
  {
    cloud_->addPoints( &points.front(), points.size() );
  }
}

void GridCellsDisplay::incomingMessage(const nav_msgs::GridCells::ConstPtr& msg)
{
  processMessage(msg);
}

void GridCellsDisplay::reset()
{
  Display::reset();
  clear();
}

void GridCellsDisplay::createProperties()
{
  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &GridCellsDisplay::getColor, this ),
                                                                      boost::bind( &GridCellsDisplay::setColor, this, _1 ), parent_category_, this );
  setPropertyHelpText(color_property_, "Color of the grid cells.");
  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &GridCellsDisplay::getAlpha, this ),
                                                                       boost::bind( &GridCellsDisplay::setAlpha, this, _1 ), parent_category_, this );
  setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the cells.");

  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &GridCellsDisplay::getTopic, this ),
                                                                                boost::bind( &GridCellsDisplay::setTopic, this, _1 ), parent_category_, this );
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<nav_msgs::GridCells>());
  setPropertyHelpText(topic_property_, "nav_msgs::GridCells topic to subscribe to.");
}

const char* GridCellsDisplay::getDescription()
{
  return "Displays data from a nav_msgs::GridCells message as either points or lines.";
}

} // namespace rviz

