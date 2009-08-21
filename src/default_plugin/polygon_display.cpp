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

#include "polygon_display.h"
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

namespace rviz
{

PolygonDisplay::PolygonDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, color_( 0.1f, 1.0f, 0.0f )
, tf_filter_(*manager->getTFClient(), "", 10, update_nh_)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "Polygon" << count++;
  manual_object_ = scene_manager_->createManualObject( ss.str() );
  manual_object_->setDynamic( true );
  scene_node_->attachObject( manual_object_ );

  setAlpha( 1.0f );

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&PolygonDisplay::incomingMessage, this, _1));
}

PolygonDisplay::~PolygonDisplay()
{
  unsubscribe();
  clear();

  scene_manager_->destroyManualObject( manual_object_ );
  scene_manager_->destroySceneNode(scene_node_->getName());
}

void PolygonDisplay::clear()
{
  manual_object_->clear();
}

void PolygonDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void PolygonDisplay::setColor( const Color& color )
{
  color_ = color;

  propertyChanged(color_property_);

  processMessage(current_message_);
  causeRender();
}

void PolygonDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  propertyChanged(alpha_property_);

  processMessage(current_message_);
  causeRender();
}

void PolygonDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(update_nh_, topic_, 10);
}

void PolygonDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void PolygonDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void PolygonDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void PolygonDisplay::fixedFrameChanged()
{
  clear();

  tf_filter_.setTargetFrame( fixed_frame_ );
}

void PolygonDisplay::update(float wall_dt, float ros_dt)
{
}

void PolygonDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg)
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

  manual_object_->clear();

  Ogre::ColourValue color( color_.r_, color_.g_, color_.b_, alpha_ );;

  uint32_t num_points = msg->polygon.points.size();
  if (num_points > 0)
  {
    manual_object_->estimateVertexCount( num_points );
    manual_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
    for( uint32_t i=0; i < num_points + 1; ++i)
    {
      Ogre::Vector3 pos(msg->polygon.points[i % num_points].x, msg->polygon.points[i % num_points].y, msg->polygon.points[i % num_points].z);
      robotToOgre(pos);
      manual_object_->position(pos);
      manual_object_->colour( color );
    }

    manual_object_->end();
  }
}

void PolygonDisplay::incomingMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
  processMessage(msg);
}

void PolygonDisplay::reset()
{
  clear();
}

void PolygonDisplay::createProperties()
{
  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &PolygonDisplay::getColor, this ),
                                                                      boost::bind( &PolygonDisplay::setColor, this, _1 ), parent_category_, this );
  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &PolygonDisplay::getAlpha, this ),
                                                                       boost::bind( &PolygonDisplay::setAlpha, this, _1 ), parent_category_, this );

  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PolygonDisplay::getTopic, this ),
                                                                                boost::bind( &PolygonDisplay::setTopic, this, _1 ), parent_category_, this );
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(geometry_msgs::PolygonStamped::__s_getDataType());
}

const char* PolygonDisplay::getDescription()
{
  return "Displays data from a geometry_msgs::PolygonStamped message as lines.";
}

} // namespace rviz

