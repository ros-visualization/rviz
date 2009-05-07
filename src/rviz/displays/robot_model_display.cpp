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

#include "robot_model_display.h"
#include "common.h"
#include "robot/robot.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include "ros/node.h"
#include "mechanism_model/robot.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

RobotModelDisplay::RobotModelDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, mechanism_topic_( "mechanism_state" )
, has_new_transforms_( false )
, has_new_mechanism_state_( false )
, time_since_last_transform_( 0.0f )
, update_rate_( 0.1f )
{
  robot_ = new Robot( vis_manager_, "Robot: " + name_ );

  setVisualVisible( true );
  setCollisionVisible( false );
  robot_->setUserData( Ogre::Any( (void*)this ) );

  setAlpha(1.0f);
}

RobotModelDisplay::~RobotModelDisplay()
{
  unsubscribe();

  delete robot_;
}

void RobotModelDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  robot_->setAlpha(alpha_);

  propertyChanged(alpha_property_);
}

void RobotModelDisplay::setRobotDescription( const std::string& description_param )
{
  description_param_ = description_param;

  propertyChanged(robot_description_property_);

  if ( isEnabled() )
  {
    load();
    causeRender();
  }
}

void RobotModelDisplay::setMechanismTopic( const std::string& topic )
{
  unsubscribe();

  mechanism_topic_ = topic;

  subscribe();

  propertyChanged(mechanism_topic_property_);
}

void RobotModelDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if ( !mechanism_topic_.empty() )
  {
    ros_node_->subscribe( mechanism_topic_, mechanism_message_, &RobotModelDisplay::incomingMechanismState, this, 1 );
  }
}

void RobotModelDisplay::unsubscribe()
{
  if ( !mechanism_topic_.empty() )
  {
    ros_node_->unsubscribe( mechanism_topic_, &RobotModelDisplay::incomingMechanismState, this );
  }
}

void RobotModelDisplay::setVisualVisible( bool visible )
{
  robot_->setVisualVisible( visible );

  propertyChanged(visual_enabled_property_);

  causeRender();
}

void RobotModelDisplay::setCollisionVisible( bool visible )
{
  robot_->setCollisionVisible( visible );

  propertyChanged(collision_enabled_property_);

  causeRender();
}

void RobotModelDisplay::setUpdateRate( float rate )
{
  update_rate_ = rate;

  propertyChanged(update_rate_property_);

  causeRender();
}

bool RobotModelDisplay::isVisualVisible()
{
  return robot_->isVisualVisible();
}

bool RobotModelDisplay::isCollisionVisible()
{
  return robot_->isCollisionVisible();
}

void RobotModelDisplay::load()
{
  std::string content;
  ros_node_->getParam(description_param_, content);

  if ( content == robot_description_ )
  {
    return;
  }

  robot_description_ = content;

  TiXmlDocument doc;
  doc.Parse(robot_description_.c_str());
  if (!doc.RootElement())
    return;

  mechanism::Robot descr;
  descr.initXml(doc.RootElement());


  robot_->load( descr );
  robot_->update( tf_, target_frame_ );
}

void RobotModelDisplay::onEnable()
{
  load();
  robot_->setVisible( true );

  subscribe();
}

void RobotModelDisplay::onDisable()
{
  unsubscribe();

  robot_->setVisible( false );
}

void RobotModelDisplay::update(float wall_dt, float ros_dt)
{
  time_since_last_transform_ += wall_dt;

  bool update = update_rate_ > 0.0001f && time_since_last_transform_ >= update_rate_;

  if ( has_new_transforms_ || update )
  {
    robot_->update( tf_, fixed_frame_ );
    causeRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }

  mechanism_message_.lock();
  if ( has_new_mechanism_state_ && update )
  {
    robot_->update( mechanism_message_ );
    has_new_mechanism_state_ = false;
  }
  mechanism_message_.unlock();
}

void RobotModelDisplay::incomingMechanismState()
{
  has_new_mechanism_state_ = true;
}

void RobotModelDisplay::targetFrameChanged()
{
  has_new_transforms_ = true;
}

void RobotModelDisplay::createProperties()
{
  visual_enabled_property_ = property_manager_->createProperty<BoolProperty>( "Visual Enabled", property_prefix_, boost::bind( &RobotModelDisplay::isVisualVisible, this ),
                                                                               boost::bind( &RobotModelDisplay::setVisualVisible, this, _1 ), category_, this );
  collision_enabled_property_ = property_manager_->createProperty<BoolProperty>( "Collision Enabled", property_prefix_, boost::bind( &RobotModelDisplay::isCollisionVisible, this ),
                                                                                 boost::bind( &RobotModelDisplay::setCollisionVisible, this, _1 ), category_, this );
  update_rate_property_ = property_manager_->createProperty<FloatProperty>( "Update Rate", property_prefix_, boost::bind( &RobotModelDisplay::getUpdateRate, this ),
                                                                                  boost::bind( &RobotModelDisplay::setUpdateRate, this, _1 ), category_, this );
  FloatPropertyPtr float_prop = update_rate_property_.lock();
  float_prop->setMin( 0.0 );

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &RobotModelDisplay::getAlpha, this ),
                                                                          boost::bind( &RobotModelDisplay::setAlpha, this, _1 ), category_, this );

  robot_description_property_ = property_manager_->createProperty<StringProperty>( "Robot Description", property_prefix_, boost::bind( &RobotModelDisplay::getRobotDescription, this ),
                                                                                   boost::bind( &RobotModelDisplay::setRobotDescription, this, _1 ), category_, this );

  mechanism_topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Mechanism Topic", property_prefix_, boost::bind( &RobotModelDisplay::getMechanismTopic, this ),
                                                                                         boost::bind( &RobotModelDisplay::setMechanismTopic, this, _1 ), category_, this );
  ROSTopicStringPropertyPtr topic_prop = mechanism_topic_property_.lock();
  topic_prop->setMessageType(robot_msgs::MechanismState::__s_getDataType());

  robot_->setPropertyManager( property_manager_, category_ );
}

void RobotModelDisplay::reset()
{
  has_new_transforms_ = true;
}

const char* RobotModelDisplay::getDescription()
{
  return "Displays a visual representation of a robot in the correct pose (as defined by the current TF transforms).";
}

} // namespace rviz

