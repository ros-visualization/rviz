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

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/parse_color.h"

#include "range_display.h"
 #include <limits>

namespace rviz
{
RangeDisplay::RangeDisplay()
{
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

  queue_size_property_ = new IntProperty( "Queue Size", 100,
                                          "Size of the tf message filter queue. It usually needs to be set at least as high as the number of sonar frames.",
                                          this, SLOT( updateQueueSize() ));
}

void RangeDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateBufferLength();
  updateColorAndAlpha();
}

RangeDisplay::~RangeDisplay()
{
  for( size_t i = 0; i < cones_.size(); i++ )
  {
    delete cones_[ i ];
  }
}

void RangeDisplay::reset()
{
  MFDClass::reset();
  updateBufferLength();
}

void RangeDisplay::updateQueueSize()
{
  tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
}

void RangeDisplay::updateColorAndAlpha()
{
  Ogre::ColourValue oc = color_property_->getOgreColor();
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

void RangeDisplay::processMessage( const sensor_msgs::Range::ConstPtr& msg )
{
  Shape* cone = cones_[ messages_received_ % buffer_length_property_->getInt() ];

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  geometry_msgs::Pose pose;
  float displayed_range = 0.0;
  if(msg->min_range <= msg->range && msg->range <= msg->max_range){
    displayed_range = msg->range;
  } else if(msg->min_range == msg->max_range){ // Fixed distance ranger
    if(msg->range < 0 && !std::isfinite(msg->range)){ // NaNs and +Inf return false here: both of those should have 0.0 as the range
      displayed_range = msg->min_range; // -Inf, display the detectable range
    }
  }
  
  pose.position.x = displayed_range/2 - .008824 * displayed_range; // .008824 fudge factor measured, must be inaccuracy of cone model.
  pose.orientation.z = 0.707;
  pose.orientation.w = 0.707;
  if( !context_->getFrameManager()->transform( msg->header.frame_id, msg->header.stamp, pose, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  cone->setPosition( position );
  cone->setOrientation( orientation );

  double cone_width = 2.0 * displayed_range * tan( msg->field_of_view / 2.0 );
  Ogre::Vector3 scale( cone_width, displayed_range, cone_width );
  cone->setScale( scale );

  QColor color = color_property_->getColor();
  cone->setColor( color.redF(), color.greenF(), color.blueF(), alpha_property_->getFloat() );
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::RangeDisplay, rviz::Display )
