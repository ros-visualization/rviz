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

#include "robot_base2d_pose_display.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"

#include "ogre_tools/arrow.h"

#include <ros/node.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

RobotBase2DPoseDisplay::RobotBase2DPoseDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, topic_( "odom" )
, color_( 1.0f, 0.1f, 0.0f )
, position_tolerance_( 0.1 )
, angle_tolerance_( 0.1 )
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  notifier_ = new tf::MessageNotifier<deprecated_msgs::RobotBase2DOdom>(tf_, ros_node_, boost::bind(&RobotBase2DPoseDisplay::incomingMessage, this, _1), "", "", 5);
}

RobotBase2DPoseDisplay::~RobotBase2DPoseDisplay()
{
  unsubscribe();

  clear();

  delete notifier_;
}

void RobotBase2DPoseDisplay::clear()
{
  V_Arrow::iterator it = arrows_.begin();
  V_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    delete *it;
  }
  arrows_.clear();

  if (last_used_message_)
  {
    last_used_message_.reset();
  }

  notifier_->clear();
}

void RobotBase2DPoseDisplay::setTopic( const std::string& topic )
{
  topic_ = topic;

  if ( isEnabled() )
  {
    notifier_->setTopic( topic );
  }

  propertyChanged(topic_property_);

  causeRender();
}

void RobotBase2DPoseDisplay::setColor( const Color& color )
{
  color_ = color;

  V_Arrow::iterator it = arrows_.begin();
  V_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    ogre_tools::Arrow* arrow = *it;
    arrow->setColor( color.r_, color.g_, color.b_, 1.0f );
  }

  propertyChanged(color_property_);

  causeRender();
}

void RobotBase2DPoseDisplay::setPositionTolerance( float tol )
{
  position_tolerance_ = tol;

  propertyChanged(position_tolerance_property_);
}

void RobotBase2DPoseDisplay::setAngleTolerance( float tol )
{
  angle_tolerance_ = tol;

  propertyChanged(angle_tolerance_property_);
}

void RobotBase2DPoseDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  notifier_->setTopic( topic_ );
}

void RobotBase2DPoseDisplay::unsubscribe()
{
  notifier_->setTopic( "" );
}

void RobotBase2DPoseDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void RobotBase2DPoseDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void RobotBase2DPoseDisplay::createProperties()
{
  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &RobotBase2DPoseDisplay::getColor, this ),
                                                                          boost::bind( &RobotBase2DPoseDisplay::setColor, this, _1 ), category_, this );
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &RobotBase2DPoseDisplay::getTopic, this ),
                                                                                boost::bind( &RobotBase2DPoseDisplay::setTopic, this, _1 ), category_, this );
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(deprecated_msgs::RobotBase2DOdom::__s_getDataType());

  position_tolerance_property_ = property_manager_->createProperty<FloatProperty>( "Position Tolerance", property_prefix_, boost::bind( &RobotBase2DPoseDisplay::getPositionTolerance, this ),
                                                                               boost::bind( &RobotBase2DPoseDisplay::setPositionTolerance, this, _1 ), category_, this );
  angle_tolerance_property_ = property_manager_->createProperty<FloatProperty>( "Angle Tolerance", property_prefix_, boost::bind( &RobotBase2DPoseDisplay::getAngleTolerance, this ),
                                                                                 boost::bind( &RobotBase2DPoseDisplay::setAngleTolerance, this, _1 ), category_, this );
}

void RobotBase2DPoseDisplay::processMessage( const MessagePtr& message )
{
  if ( last_used_message_ )
  {
    if ( abs(last_used_message_->pos.x - message->pos.x) < position_tolerance_
      && abs(last_used_message_->pos.y - message->pos.y) < position_tolerance_
      && abs(last_used_message_->pos.th - message->pos.th) < angle_tolerance_ )
    {
      return;
    }
  }

  ogre_tools::Arrow* arrow = new ogre_tools::Arrow( scene_manager_, scene_node_, 0.8f, 0.05f, 0.2f, 0.2f );

  transformArrow( message, arrow );

  arrow->setColor( color_.r_, color_.g_, color_.b_, 1.0f );
  arrow->setUserData( Ogre::Any((void*)this) );

  arrows_.push_back( arrow );
  last_used_message_ = message;
}

void RobotBase2DPoseDisplay::transformArrow( const MessagePtr& message, ogre_tools::Arrow* arrow )
{
  std::string frame_id = message->header.frame_id;
  if ( frame_id.empty() )
  {
    frame_id = fixed_frame_;
  }

  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( message->pos.th, 0.0f, 0.0f ), btVector3( message->pos.x, message->pos.y, 0.0f ) ),
                              message->header.stamp, frame_id );

  try
  {
    tf_->transformPose( fixed_frame_, pose, pose );
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR( "Error transforming 2d base pose '%s' from frame '%s' to frame '%s'\n", name_.c_str(), message->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  btScalar yaw, pitch, roll;
  pose.getBasis().getEulerZYX( yaw, pitch, roll );
  Ogre::Matrix3 orient;
  orient.FromEulerAnglesZXY( Ogre::Radian( roll ), Ogre::Radian( pitch ), Ogre::Radian( yaw ) );
  arrow->setOrientation( orient );

  Ogre::Vector3 pos( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  robotToOgre( pos );
  arrow->setPosition( pos );
}

void RobotBase2DPoseDisplay::targetFrameChanged()
{
}

void RobotBase2DPoseDisplay::fixedFrameChanged()
{
  notifier_->setTargetFrame( fixed_frame_ );
  clear();
}

void RobotBase2DPoseDisplay::update( float dt )
{
  V_RobotBase2DOdom local_queue;

  {
    boost::mutex::scoped_lock lock(queue_mutex_);

    local_queue.swap( message_queue_ );
  }

  if ( !local_queue.empty() )
  {
    V_RobotBase2DOdom::iterator it = local_queue.begin();
    V_RobotBase2DOdom::iterator end = local_queue.end();
    for ( ; it != end; ++it )
    {
      processMessage( *it );
    }

    causeRender();
  }
}

void RobotBase2DPoseDisplay::incomingMessage( const MessagePtr& message )
{
  boost::mutex::scoped_lock lock(queue_mutex_);

  message_queue_.push_back( message );
}

void RobotBase2DPoseDisplay::reset()
{
  clear();
}

const char* RobotBase2DPoseDisplay::getDescription()
{
  return "Accumulates and displays poses from a deprecated_msgs::RobotBase2DOdom message.";
}

} // namespace rviz
