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

#include "odometry_display.h"
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

namespace rviz
{

OdometryDisplay::OdometryDisplay()
  : Display()
  , color_( 1.0f, 0.1f, 0.0f )
  , keep_(100)
  , length_( 1.0 )
  , position_tolerance_( 0.1 )
  , angle_tolerance_( 0.1 )
  , messages_received_(0)
{
}

OdometryDisplay::~OdometryDisplay()
{
  unsubscribe();

  clear();

  delete tf_filter_;
}

void OdometryDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(*vis_manager_->getTFClient(), "", 5, update_nh_);
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&OdometryDisplay::incomingMessage, this, _1));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

void OdometryDisplay::clear()
{
  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    delete *it;
  }
  arrows_.clear();

  if (last_used_message_)
  {
    last_used_message_.reset();
  }

  tf_filter_->clear();

  messages_received_ = 0;
  setStatus(status_levels::Warn, "Topic", "No messages received");
}

void OdometryDisplay::setTopic( const std::string& topic )
{
  unsubscribe();
  topic_ = topic;
  clear();
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void OdometryDisplay::setColor( const Color& color )
{
  color_ = color;

  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    Arrow* arrow = *it;
    arrow->setColor( color.r_, color.g_, color.b_, 1.0f );
  }

  propertyChanged(color_property_);

  causeRender();
}

void OdometryDisplay::setLength( float length )
{
  length_ = length;
  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  Ogre::Vector3 scale( length_, length_, length_ );
  for ( ; it != end; ++it )
  {
    Arrow* arrow = *it;
    arrow->setScale( scale );
  }
  propertyChanged( length_property_ );
  causeRender();
}

void OdometryDisplay::setKeep(uint32_t keep)
{
  keep_ = keep;

  propertyChanged(keep_property_);
}

void OdometryDisplay::setPositionTolerance( float tol )
{
  position_tolerance_ = tol;

  propertyChanged(position_tolerance_property_);
}

void OdometryDisplay::setAngleTolerance( float tol )
{
  angle_tolerance_ = tol;

  propertyChanged(angle_tolerance_property_);
}

void OdometryDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe(update_nh_, topic_, 5);
    setStatus(status_levels::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
  }
}

void OdometryDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void OdometryDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void OdometryDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void OdometryDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &OdometryDisplay::getTopic, this ),
                                                                                boost::bind( &OdometryDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "nav_msgs::Odometry topic to subscribe to.");
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<nav_msgs::Odometry>());

  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &OdometryDisplay::getColor, this ),
                                                                          boost::bind( &OdometryDisplay::setColor, this, _1 ), parent_category_, this );
  setPropertyHelpText(color_property_, "Color of the arrows.");

  position_tolerance_property_ = property_manager_->createProperty<FloatProperty>( "Position Tolerance", property_prefix_, boost::bind( &OdometryDisplay::getPositionTolerance, this ),
                                                                               boost::bind( &OdometryDisplay::setPositionTolerance, this, _1 ), parent_category_, this );
  setPropertyHelpText(position_tolerance_property_, "Distance, in meters from the last arrow dropped, that will cause a new arrow to drop.");
  angle_tolerance_property_ = property_manager_->createProperty<FloatProperty>( "Angle Tolerance", property_prefix_, boost::bind( &OdometryDisplay::getAngleTolerance, this ),
                                                                                 boost::bind( &OdometryDisplay::setAngleTolerance, this, _1 ), parent_category_, this );
  setPropertyHelpText(angle_tolerance_property_, "Angular distance from the last arrow dropped, that will cause a new arrow to drop.");

  keep_property_ = property_manager_->createProperty<IntProperty>( "Keep", property_prefix_, boost::bind( &OdometryDisplay::getKeep, this ),
                                                                               boost::bind( &OdometryDisplay::setKeep, this, _1 ), parent_category_, this );
  setPropertyHelpText(keep_property_, "Number of arrows to keep before removing the oldest.");

  length_property_ = property_manager_->createProperty<FloatProperty>( "Length", property_prefix_, boost::bind( &OdometryDisplay::getLength, this ),
                                                                       boost::bind( &OdometryDisplay::setLength, this, _1 ), parent_category_, this );
  setPropertyHelpText(length_property_, "Length of each arrow.");
}

bool validateFloats(const nav_msgs::Odometry& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.pose.pose);
  valid = valid && validateFloats(msg.twist.twist);
  return valid;
}

void OdometryDisplay::processMessage( const nav_msgs::Odometry::ConstPtr& message )
{
  ++messages_received_;

  if (!validateFloats(*message))
  {
    setStatus(status_levels::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
    return;
  }

  {
    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus(status_levels::Ok, "Topic", ss.str());
  }

  if ( last_used_message_ )
  {
    Ogre::Vector3 last_position(last_used_message_->pose.pose.position.x, last_used_message_->pose.pose.position.y, last_used_message_->pose.pose.position.z);
    Ogre::Vector3 current_position(message->pose.pose.position.x, message->pose.pose.position.y, message->pose.pose.position.z);
    Ogre::Quaternion last_orientation(last_used_message_->pose.pose.orientation.w, last_used_message_->pose.pose.orientation.x, last_used_message_->pose.pose.orientation.y, last_used_message_->pose.pose.orientation.z);
    Ogre::Quaternion current_orientation(message->pose.pose.orientation.w, message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z);

    if ((last_position - current_position).length() < position_tolerance_ && (last_orientation - current_orientation).normalise() < angle_tolerance_)
    {
      return;
    }
  }

  Arrow* arrow = new Arrow( scene_manager_, scene_node_, 0.8f, 0.05f, 0.2f, 0.2f );

  transformArrow( message, arrow );

  arrow->setColor( color_.r_, color_.g_, color_.b_, 1.0f );
  Ogre::Vector3 scale( length_, length_, length_ );
  arrow->setScale( scale );
  arrow->setUserData( Ogre::Any((void*)this) );

  arrows_.push_back( arrow );
  last_used_message_ = message;
}

void OdometryDisplay::transformArrow( const nav_msgs::Odometry::ConstPtr& message, Arrow* arrow )
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!vis_manager_->getFrameManager()->transform(message->header, message->pose.pose, position, orientation))
  {
    ROS_ERROR( "Error transforming odometry '%s' from frame '%s' to frame '%s'", name_.c_str(), message->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  arrow->setPosition( position );

  // Arrow points in -Z direction, so rotate the orientation before display.
  // TODO: is it safe to change Arrow to point in +X direction?
  arrow->setOrientation( orientation * Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y ));
}

void OdometryDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_ );
  clear();
}

void OdometryDisplay::update(float wall_dt, float ros_dt)
{
  if (keep_ > 0)
  {
    while (arrows_.size() > keep_)
    {
      delete arrows_.front();
      arrows_.pop_front();
    }
  }
}

void OdometryDisplay::incomingMessage( const nav_msgs::Odometry::ConstPtr& message )
{
  processMessage(message);
  causeRender();
}

void OdometryDisplay::reset()
{
  Display::reset();

  clear();
}

} // namespace rviz
