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


#include <boost/bind.hpp>

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "odometry_display.h"

namespace rviz
{

OdometryDisplay::OdometryDisplay()
  : Display()
  , messages_received_(0)
{
  topic_property_ = new RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<nav_msgs::Odometry>() ),
                                          "nav_msgs::Odometry topic to subscribe to.",
                                          this, SLOT( updateTopic() ));

  color_property_ = new ColorProperty( "Color", QColor( 255, 25, 0 ),
                                       "Color of the arrows.",
                                       this, SLOT( updateColor() ));

  position_tolerance_property_ = new FloatProperty( "Position Tolerance", .1,
                                                    "Distance, in meters from the last arrow dropped, "
                                                    "that will cause a new arrow to drop.",
                                                    this );
  position_tolerance_property_->setMin( 0 );
                                                
  angle_tolerance_property_ = new FloatProperty( "Angle Tolerance", .1,
                                                 "Angular distance from the last arrow dropped, "
                                                 "that will cause a new arrow to drop.",
                                                 this );
  angle_tolerance_property_->setMin( 0 );

  keep_property_ = new IntProperty( "Keep", 100,
                                    "Number of arrows to keep before removing the oldest.  0 means keep all of them.",
                                    this );
  keep_property_->setMin( 0 );

  length_property_ = new FloatProperty( "Length", 1.0,
                                        "Length of each arrow.",
                                        this, SLOT( updateLength() ));
}

OdometryDisplay::~OdometryDisplay()
{
  if ( initialized() )
  {
    unsubscribe();
    clear();
    delete tf_filter_;
  }
}

void OdometryDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>( *context_->getTFClient(), fixed_frame_.toStdString(),
                                                          5, update_nh_ );

  tf_filter_->connectInput( sub_ );
  tf_filter_->registerCallback( boost::bind( &OdometryDisplay::incomingMessage, this, _1 ));
  context_->getFrameManager()->registerFilterForTransformStatusCheck( tf_filter_, this );
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

  if( last_used_message_ )
  {
    last_used_message_.reset();
  }

  tf_filter_->clear();

  messages_received_ = 0;
  setStatus( StatusProperty::Warn, "Topic", "No messages received" );
}

void OdometryDisplay::updateTopic()
{
  unsubscribe();
  clear();
  subscribe();
  context_->queueRender();
}

void OdometryDisplay::updateColor()
{
  QColor color = color_property_->getColor();
  float red   = color.redF();
  float green = color.greenF();
  float blue  = color.blueF();

  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for( ; it != end; ++it )
  {
    Arrow* arrow = *it;
    arrow->setColor( red, green, blue, 1.0f );
  }
  context_->queueRender();
}

void OdometryDisplay::updateLength()
{
  float length = length_property_->getFloat();
  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  Ogre::Vector3 scale( length, length, length );
  for ( ; it != end; ++it )
  {
    Arrow* arrow = *it;
    arrow->setScale( scale );
  }
  context_->queueRender();
}

void OdometryDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe( update_nh_, topic_property_->getTopicStd(), 5 );
    setStatus( StatusProperty::Ok, "Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
  }
}

void OdometryDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void OdometryDisplay::onEnable()
{
  subscribe();
}

void OdometryDisplay::onDisable()
{
  unsubscribe();
  clear();
}

bool validateFloats(const nav_msgs::Odometry& msg)
{
  bool valid = true;
  valid = valid && validateFloats( msg.pose.pose );
  valid = valid && validateFloats( msg.twist.twist );
  return valid;
}

void OdometryDisplay::incomingMessage( const nav_msgs::Odometry::ConstPtr& message )
{
  ++messages_received_;

  if( !validateFloats( *message ))
  {
    setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  setStatus( StatusProperty::Ok, "Topic", QString::number( messages_received_ ) + " messages received" );

  if( last_used_message_ )
  {
    Ogre::Vector3 last_position(last_used_message_->pose.pose.position.x, last_used_message_->pose.pose.position.y, last_used_message_->pose.pose.position.z);
    Ogre::Vector3 current_position(message->pose.pose.position.x, message->pose.pose.position.y, message->pose.pose.position.z);
    Ogre::Quaternion last_orientation(last_used_message_->pose.pose.orientation.w, last_used_message_->pose.pose.orientation.x, last_used_message_->pose.pose.orientation.y, last_used_message_->pose.pose.orientation.z);
    Ogre::Quaternion current_orientation(message->pose.pose.orientation.w, message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z);

    if( (last_position - current_position).length() < position_tolerance_property_->getFloat() &&
        (last_orientation - current_orientation).normalise() < angle_tolerance_property_->getFloat() )
    {
      return;
    }
  }

  Arrow* arrow = new Arrow( scene_manager_, scene_node_, 0.8f, 0.05f, 0.2f, 0.2f );

  transformArrow( message, arrow );

  QColor color = color_property_->getColor();
  arrow->setColor( color.redF(), color.greenF(), color.blueF(), 1.0f );

  float length = length_property_->getFloat();
  Ogre::Vector3 scale( length, length, length );
  arrow->setScale( scale );

  arrows_.push_back( arrow );

  last_used_message_ = message;
  context_->queueRender();
}

void OdometryDisplay::transformArrow( const nav_msgs::Odometry::ConstPtr& message, Arrow* arrow )
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->transform( message->header, message->pose.pose, position, orientation ))
  {
    ROS_ERROR( "Error transforming odometry '%s' from frame '%s' to frame '%s'",
               qPrintable( getName() ), message->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  arrow->setPosition( position );

  // Arrow points in -Z direction, so rotate the orientation before display.
  // TODO: is it safe to change Arrow to point in +X direction?
  arrow->setOrientation( orientation * Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y ));
}

void OdometryDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_.toStdString() );
  clear();
}

void OdometryDisplay::update( float wall_dt, float ros_dt )
{
  size_t keep = keep_property_->getInt();
  if( keep > 0 )
  {
    while( arrows_.size() > keep )
    {
      delete arrows_.front();
      arrows_.pop_front();
    }
  }
}

void OdometryDisplay::reset()
{
  Display::reset();
  clear();
}

void OdometryDisplay::setTopic( const QString &topic, const QString &datatype )
{
  topic_property_->setString( topic );
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::OdometryDisplay, rviz::Display )
