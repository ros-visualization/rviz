/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/parse_color.h"
#include "rviz/properties/ros_topic_property.h"

#include "range_display.h"

namespace rviz
{
RangeDisplay::RangeDisplay()
  : Display()
  , messages_received_( 0 )
{
  topic_property_ = new RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<sensor_msgs::Range>() ),
                                          "sensor_msgs::Range topic to subscribe to.",
                                          this, SLOT( updateTopic() ));

  color_property_ = new ColorProperty( "Color", Qt::white,
                                       "Color to draw the range.",
                                       this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new FloatProperty( "Alpha", 0.5,
                                       "Amount of transparency to apply to the range.",
                                       this, SLOT( updateColorAndAlpha() ));

  buffer_length_property_ = new IntProperty( "Buffer Length", 1,
                                             "Number of prior measurements to display.",
                                             this, SLOT( updateBufferLength() ));
  buffer_length_property_->setMin( 1 );
}

void RangeDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<sensor_msgs::Range>( *context_->getTFClient(),
                                                          fixed_frame_.toStdString(), 10, update_nh_ );

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->setVisible( false );
  
  updateBufferLength();

  tf_filter_->connectInput( sub_ );
  tf_filter_->registerCallback( boost::bind( &RangeDisplay::incomingMessage, this, _1 ));
  context_->getFrameManager()->registerFilterForTransformStatusCheck( tf_filter_, this );

  updateColorAndAlpha();
}

RangeDisplay::~RangeDisplay()
{
  unsubscribe();
  clear();
  for( size_t i = 0; i < cones_.size(); i++ )
  {
    delete cones_[ i ];
  }

  delete tf_filter_;
}

void RangeDisplay::clear()
{
  updateBufferLength();
  tf_filter_->clear();
  messages_received_ = 0;
  setStatus( StatusProperty::Warn, "Topic", "No messages received" );
}

void RangeDisplay::updateTopic()
{
  unsubscribe();
  clear();
  subscribe();
  context_->queueRender();
}

void RangeDisplay::updateColorAndAlpha()
{
  Ogre::ColourValue oc = qtToOgre( color_property_->getColor() );
  float alpha = alpha_property_->getFloat();
  for( size_t i = 0; i < cones_.size(); i++ )
  {
    cones_[i]->setColor( oc.r, oc.g, oc.b, alpha );
  }
  context_->queueRender();
}

void RangeDisplay::updateBufferLength()
{
  int buffer_length = buffer_length_property_->getInt();
  QColor color = color_property_->getColor();

  for( size_t i = 0; i < cones_.size(); i++ )
  {
    delete cones_[i];
  }
  cones_.resize( buffer_length );
  for( size_t i = 0; i < cones_.size(); i++ )
  {
    Shape* cone = new Shape( Shape::Cone, context_->getSceneManager(), scene_node_ );
    cones_[ i ] = cone;    

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    Ogre::Vector3 scale( 0, 0, 0 );
    cone->setScale( scale );
    cone->setColor( color.redF(), color.greenF(), color.blueF(), 0 );
  }
}

void RangeDisplay::subscribe()
{
  if( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe( update_nh_, topic_property_->getTopicStd(), 10 );
    setStatus( StatusProperty::Ok, "Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
  }
}

void RangeDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void RangeDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void RangeDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void RangeDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_.toStdString() );
  clear();
}

void RangeDisplay::incomingMessage( const sensor_msgs::Range::ConstPtr& msg )
{
  if( !msg )
  {
    return;
  }

  ++messages_received_;
  
  Shape* cone = cones_[ messages_received_ % buffer_length_property_->getInt() ];

  {
    setStatus( StatusProperty::Ok, "Topic", QString::number( messages_received_ ) + " messages received" );
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  geometry_msgs::Pose pose;
  pose.position.x = msg->range/2 - .008824 * msg->range; // .008824 fudge factor measured, must be inaccuracy of cone model.
  pose.orientation.z = 0.707;
  pose.orientation.w = 0.707;
  if( !context_->getFrameManager()->transform( msg->header.frame_id, msg->header.stamp, pose, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  cone->setPosition( position );
  cone->setOrientation( orientation );

  double cone_width = 2.0 * msg->range * tan( msg->field_of_view / 2.0 );
  Ogre::Vector3 scale( cone_width, msg->range, cone_width );
  cone->setScale( scale );

  QColor color = color_property_->getColor();
  cone->setColor( color.redF(), color.greenF(), color.blueF(), alpha_property_->getFloat() );
}

void RangeDisplay::reset()
{
  Display::reset();
  clear();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( rviz, Range, rviz::RangeDisplay, rviz::Display )
