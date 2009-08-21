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
#include "rviz/common.h"

#include "ogre_tools/arrow.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <ogre_tools/point_cloud.h>

namespace rviz
{

GridCellsDisplay::GridCellsDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, color_( 0.1f, 1.0f, 0.0f )
, tf_filter_(*manager->getTFClient(), "", 10, update_nh_)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "PolyLine" << count++;

  cloud_ = new ogre_tools::PointCloud();
  cloud_->setRenderMode( ogre_tools::PointCloud::RM_BILLBOARDS_COMMON_FACING );
  cloud_->setCommonDirection( Ogre::Vector3::UNIT_Y );
  cloud_->setCommonUpVector( Ogre::Vector3::NEGATIVE_UNIT_Z );
  scene_node_->attachObject(cloud_);
  setAlpha( 1.0f );

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&GridCellsDisplay::incomingMessage, this, _1));
}

GridCellsDisplay::~GridCellsDisplay()
{
  unsubscribe();
  clear();

  scene_manager_->destroySceneNode(scene_node_->getName());
  delete cloud_;
}

void GridCellsDisplay::clear()
{
  cloud_->clear();
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

  sub_.subscribe(update_nh_, topic_, 10);
}

void GridCellsDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void GridCellsDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void GridCellsDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void GridCellsDisplay::fixedFrameChanged()
{
  clear();

  tf_filter_.setTargetFrame( fixed_frame_ );
}

void GridCellsDisplay::update(float wall_dt, float ros_dt)
{
}

void GridCellsDisplay::processMessage(const nav_msgs::GridCells::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  clear();

  std::string frame_id = msg->header.frame_id;
  if (frame_id.empty())
  {
    frame_id = "/map";
  }

  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0.0f, 0.0f, 0.0f ), btVector3( 0.0f, 0.0f, 0.0f ) ),
                                ros::Time(), frame_id);

  if (vis_manager_->getTFClient()->canTransform(fixed_frame_, frame_id, ros::Time()))
  {
    try
    {
      vis_manager_->getTFClient()->transformPose( fixed_frame_, pose, pose );
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming from frame 'map' to frame '%s'", fixed_frame_.c_str() );
    }
  }

  Ogre::Vector3 position = Ogre::Vector3( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  robotToOgre( position );

  btQuaternion quat;
  pose.getBasis().getRotation( quat );
  Ogre::Quaternion orientation( Ogre::Quaternion::IDENTITY );
  ogreToRobot( orientation );
  orientation = Ogre::Quaternion( quat.w(), quat.x(), quat.y(), quat.z() ) * orientation;
  robotToOgre( orientation );

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  Ogre::ColourValue color( color_.r_, color_.g_, color_.b_, alpha_ );

  cloud_->setDimensions(msg->cell_width, msg->cell_height, 0.0);

  uint32_t num_points = msg->cells.size();

  typedef std::vector< ogre_tools::PointCloud::Point > V_Point;
  V_Point points;
  points.resize( num_points );
  for(uint32_t i = 0; i < num_points; i++)
  {
    ogre_tools::PointCloud::Point& current_point = points[ i ];

    Ogre::Vector3 pos(msg->cells[i].x, msg->cells[i].y, msg->cells[i].z);
    robotToOgre(pos);

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
  clear();
}

void GridCellsDisplay::createProperties()
{
  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &GridCellsDisplay::getColor, this ),
                                                                      boost::bind( &GridCellsDisplay::setColor, this, _1 ), parent_category_, this );
  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &GridCellsDisplay::getAlpha, this ),
                                                                       boost::bind( &GridCellsDisplay::setAlpha, this, _1 ), parent_category_, this );

  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &GridCellsDisplay::getTopic, this ),
                                                                                boost::bind( &GridCellsDisplay::setTopic, this, _1 ), parent_category_, this );
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(nav_msgs::GridCells::__s_getDataType());
}

const char* GridCellsDisplay::getDescription()
{
  return "Displays data from a nav_msgs::GridCells message as either points or lines.";
}

} // namespace rviz

