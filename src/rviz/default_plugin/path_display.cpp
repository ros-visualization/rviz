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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/validate_floats.h"

#include "rviz/default_plugin/path_display.h"

namespace rviz
{

PathDisplay::PathDisplay()
  : manual_object_( NULL )
{
  color_property_ = new ColorProperty( "Color", QColor( 25, 255, 0 ),
                                       "Color to draw the path.", this );

  alpha_property_ = new FloatProperty( "Alpha", 1.0,
                                       "Amount of transparency to apply to the path.", this );
}

PathDisplay::~PathDisplay()
{
  if( manual_object_ )
  {
    manual_object_->clear();
    scene_manager_->destroyManualObject( manual_object_ );
  }
}

void PathDisplay::onInitialize()
{
  MFDClass::onInitialize();

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic( true );
  scene_node_->attachObject( manual_object_ );
}

void PathDisplay::reset()
{
  MFDClass::reset();
  manual_object_->clear();
}

bool validateFloats( const nav_msgs::Path& msg )
{
  bool valid = true;
  valid = valid && validateFloats( msg.poses );
  return valid;
}

void PathDisplay::processMessage( const nav_msgs::Path::ConstPtr& msg )
{
  if( !validateFloats( *msg ))
  {
    setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  manual_object_->clear();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->getTransform( msg->header, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  manual_object_->clear();

  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  uint32_t num_points = msg->poses.size();
  manual_object_->estimateVertexCount( num_points );
  manual_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
  for( uint32_t i=0; i < num_points; ++i)
  {
    const geometry_msgs::Point& pos = msg->poses[ i ].pose.position;
    manual_object_->position( pos.x, pos.y, pos.z );
    manual_object_->colour( color );
  }

  manual_object_->end();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( rviz, Path, rviz::PathDisplay, rviz::Display )
