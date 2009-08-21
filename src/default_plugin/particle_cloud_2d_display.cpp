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

#include "particle_cloud_2d_display.h"
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

namespace rviz
{

ParticleCloud2DDisplay::ParticleCloud2DDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, topic_( "particlecloud" )
, color_( 1.0f, 0.1f, 0.0f )
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "ParticleCloud2D" << count++;
  manual_object_ = scene_manager_->createManualObject( ss.str() );
  manual_object_->setDynamic( true );
  scene_node_->attachObject( manual_object_ );
}

ParticleCloud2DDisplay::~ParticleCloud2DDisplay()
{
  unsubscribe();
  clear();

#if 0
  V_Arrow::iterator it = arrows_.begin();
  V_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    ogre_tools::Arrow* arrow = *it;
    delete arrow;
  }

  arrows_.clear();
#endif

  scene_manager_->destroyManualObject( manual_object_ );
}

void ParticleCloud2DDisplay::clear()
{
#if 0
  V_Arrow::iterator it = arrows_.begin();
  V_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    ogre_tools::Arrow* arrow = *it;
    arrow->getSceneNode()->setVisible( false );
  }

  arrow_count_ = 0;
#endif

  manual_object_->clear();
}

void ParticleCloud2DDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void ParticleCloud2DDisplay::setColor( const Color& color )
{
  color_ = color;

#if 0
  V_Arrow::iterator it = arrows_.begin();
  V_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    ogre_tools::Arrow* arrow = *it;
    arrow->setColor( color.r_, color.g_, color.b_, 1.0f );
  }
#endif

  propertyChanged(color_property_);

  causeRender();
}

void ParticleCloud2DDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if (!topic_.empty())
  {
    sub_ = update_nh_.subscribe(topic_, 1, &ParticleCloud2DDisplay::incomingMessage, this);
  }
}

void ParticleCloud2DDisplay::unsubscribe()
{
  sub_.shutdown();
}

void ParticleCloud2DDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void ParticleCloud2DDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void ParticleCloud2DDisplay::createProperties()
{
  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &ParticleCloud2DDisplay::getColor, this ),
                                                                          boost::bind( &ParticleCloud2DDisplay::setColor, this, _1 ), parent_category_, this );
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &ParticleCloud2DDisplay::getTopic, this ),
                                                                                boost::bind( &ParticleCloud2DDisplay::setTopic, this, _1 ), parent_category_, this );
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(geometry_msgs::PoseArray::__s_getDataType());
}

void ParticleCloud2DDisplay::fixedFrameChanged()
{
  clear();
}

void ParticleCloud2DDisplay::update(float wall_dt, float ros_dt)
{
}

void ParticleCloud2DDisplay::processMessage(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  clear();

  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0.0f, 0.0f, 0.0f ), btVector3( 0.0f, 0.0f, 0.0f ) ),
                                ros::Time(), "map" );

  if (vis_manager_->getTFClient()->canTransform(fixed_frame_, "map", ros::Time()))
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

#if 0
  uint32_t particle_count = msg->poses.size();
  for ( uint32_t i = 0; i < particle_count; ++i )
  {
    Ogre::Vector3 pos( -msg->poses[i].y, 0.0f, -msg->poses[i].x );
    Ogre::Quaternion orient( Ogre::Quaternion( Ogre::Radian( msg->poses[i].th ), Ogre::Vector3::UNIT_Y ) );

    ogre_tools::Arrow* arrow = NULL;
    if ( particle_count > arrows_.size() )
    {
      arrow = new ogre_tools::Arrow( scene_manager_, scene_node_, 0.8f, 0.02f, 0.2f, 0.1f );
      arrow->setColor( color_.r_, color_.g_, color_.b_, 1.0f );
      arrows_.push_back( arrow );
      ++arrow_count_;
    }
    else
    {
      arrow = arrows_[ arrow_count_++ ];
    }

    arrow->setPosition( pos );
    arrow->setOrientation( orient );
    arrow->getSceneNode()->setVisible( true );
  }
#endif

  manual_object_->clear();

  Ogre::ColourValue color( color_.r_, color_.g_, color_.b_, 1.0f );
  int num_poses = msg->get_poses_size();
  manual_object_->estimateVertexCount( num_poses * 8 );
  manual_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
  for( int i=0; i < num_poses; ++i)
  {
    Ogre::Vector3 pos( -msg->poses[i].position.y, 0.0f, -msg->poses[i].position.x );
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(msg->poses[i].orientation, orientation);
    double yaw, pitch, roll;
    btMatrix3x3(orientation).getEulerZYX(yaw, pitch, roll);
    Ogre::Quaternion orient( Ogre::Quaternion( Ogre::Radian( yaw ), Ogre::Vector3::UNIT_Y ) );

    const static float radius = 0.3f;
    Ogre::Vector3 vertices[8];
    vertices[0] = pos;
    vertices[1] = pos + orient * Ogre::Vector3(0.0f, 0.0f, -radius);
    vertices[2] = vertices[1];
    vertices[3] = pos + orient * Ogre::Vector3(0.25*radius, 0.0f, -0.75*radius);
    vertices[4] = vertices[3];
    vertices[5] = pos + orient * Ogre::Vector3(-0.25*radius, 0.0f, -0.75*radius);
    vertices[6] = vertices[5];
    vertices[7] = pos + orient * Ogre::Vector3(0.0f, 0.0f, -radius);

    for ( int i = 0; i < 8; ++i )
    {
      manual_object_->position( vertices[i] );
      manual_object_->colour( color );
    }
  }
  manual_object_->end();

  causeRender();
}

void ParticleCloud2DDisplay::incomingMessage(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  processMessage(msg);
}

void ParticleCloud2DDisplay::reset()
{
  clear();
}

} // namespace rviz

