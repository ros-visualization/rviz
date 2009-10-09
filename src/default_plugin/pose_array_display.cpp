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

#include "pose_array_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/common.h"
#include "rviz/frame_manager.h"

#include "ogre_tools/arrow.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>

namespace rviz
{

PoseArrayDisplay::PoseArrayDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, color_( 1.0f, 0.1f, 0.0f )
, messages_received_(0)
, tf_filter_(*manager->getTFClient(), "", 2, update_nh_)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "ParticleCloud2D" << count++;
  manual_object_ = scene_manager_->createManualObject( ss.str() );
  manual_object_->setDynamic( true );
  scene_node_->attachObject( manual_object_ );

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&PoseArrayDisplay::incomingMessage, this, _1));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

PoseArrayDisplay::~PoseArrayDisplay()
{
  unsubscribe();
  clear();

  scene_manager_->destroyManualObject( manual_object_ );
}

void PoseArrayDisplay::clear()
{
  manual_object_->clear();

  messages_received_ = 0;
  setStatus(StatusProperty::Warn, "Topic", "No messages received");
}

void PoseArrayDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void PoseArrayDisplay::setColor( const Color& color )
{
  color_ = color;

  propertyChanged(color_property_);

  causeRender();
}

void PoseArrayDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(update_nh_, topic_, 5);
}

void PoseArrayDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void PoseArrayDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void PoseArrayDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void PoseArrayDisplay::createProperties()
{
  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &PoseArrayDisplay::getColor, this ),
                                                                          boost::bind( &PoseArrayDisplay::setColor, this, _1 ), parent_category_, this );
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PoseArrayDisplay::getTopic, this ),
                                                                                boost::bind( &PoseArrayDisplay::setTopic, this, _1 ), parent_category_, this );
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(geometry_msgs::PoseArray::__s_getDataType());
}

void PoseArrayDisplay::fixedFrameChanged()
{
  clear();
  tf_filter_.setTargetFrame( fixed_frame_ );
}

void PoseArrayDisplay::update(float wall_dt, float ros_dt)
{
}

void PoseArrayDisplay::processMessage(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  ++messages_received_;
  {
    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus(StatusProperty::Ok, "Topic", ss.str());
  }

  manual_object_->clear();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!vis_manager_->getFrameManager()->getTransform(msg->header, position, orientation, true))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  manual_object_->clear();

  Ogre::ColourValue color( color_.r_, color_.g_, color_.b_, 1.0f );
  int num_poses = msg->get_poses_size();
  manual_object_->estimateVertexCount( num_poses * 8 );
  manual_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
  for( int i=0; i < num_poses; ++i)
  {
    Ogre::Vector3 pos(msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z);
    robotToOgre(pos);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->poses[i].orientation, quat);
    Ogre::Quaternion orient = Ogre::Quaternion::IDENTITY;
    ogreToRobot( orient );
    orient = Ogre::Quaternion( quat.w(), quat.x(), quat.y(), quat.z() ) * orient;
    robotToOgre(orient);

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

void PoseArrayDisplay::incomingMessage(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  processMessage(msg);
}

void PoseArrayDisplay::reset()
{
  clear();
}

} // namespace rviz

