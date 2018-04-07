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

#include <OgreEntity.h>
#include <OgreSceneNode.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/validate_quaternions.h"

#include "rviz/default_plugin/pose_display.h"

namespace rviz
{

class PoseDisplaySelectionHandler: public SelectionHandler
{
public:
  PoseDisplaySelectionHandler( PoseDisplay* display, DisplayContext* context )
    : SelectionHandler( context )
    , display_( display )
  {}

  void createProperties( const Picked& obj, Property* parent_property )
  {
    Property* cat = new Property( "Pose " + display_->getName(), QVariant(), "", parent_property );
    properties_.push_back( cat );

    frame_property_ = new StringProperty( "Frame", "", "", cat );
    frame_property_->setReadOnly( true );

    position_property_ = new VectorProperty( "Position", Ogre::Vector3::ZERO, "", cat );
    position_property_->setReadOnly( true );

    orientation_property_ = new QuaternionProperty( "Orientation", Ogre::Quaternion::IDENTITY, "", cat );
    orientation_property_->setReadOnly( true );
  }

  void getAABBs( const Picked& obj, V_AABB& aabbs )
  {
    if( display_->pose_valid_ )
    {
      if( display_->shape_property_->getOptionInt() == PoseDisplay::Arrow )
      {
        aabbs.push_back( display_->arrow_->getHead()->getEntity()->getWorldBoundingBox() );
        aabbs.push_back( display_->arrow_->getShaft()->getEntity()->getWorldBoundingBox() );
      }
      else
      {
        aabbs.push_back( display_->axes_->getXShape()->getEntity()->getWorldBoundingBox() );
        aabbs.push_back( display_->axes_->getYShape()->getEntity()->getWorldBoundingBox() );
        aabbs.push_back( display_->axes_->getZShape()->getEntity()->getWorldBoundingBox() );
      }
    }
  }

  void setMessage(const geometry_msgs::PoseStampedConstPtr& message)
  {
    // properties_.size() should only be > 0 after createProperties()
    // and before destroyProperties(), during which frame_property_,
    // position_property_, and orientation_property_ should be valid
    // pointers.
    if( properties_.size() > 0 )
    {
      frame_property_->setStdString( message->header.frame_id );
      position_property_->setVector( Ogre::Vector3( message->pose.position.x,
                                                    message->pose.position.y,
                                                    message->pose.position.z ));
      orientation_property_->setQuaternion( Ogre::Quaternion( message->pose.orientation.w,
                                                              message->pose.orientation.x,
                                                              message->pose.orientation.y,
                                                              message->pose.orientation.z ));
    }
  }

private:
  PoseDisplay* display_;
  StringProperty* frame_property_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;
};

PoseDisplay::PoseDisplay()
  : pose_valid_( false )
{
  shape_property_ = new EnumProperty( "Shape", "Arrow", "Shape to display the pose as.",
                                      this, SLOT( updateShapeChoice() ));
  shape_property_->addOption( "Arrow", Arrow );
  shape_property_->addOption( "Axes", Axes );

  color_property_ = new ColorProperty( "Color", QColor( 255, 25, 0 ), "Color to draw the arrow.",
                                       this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new FloatProperty( "Alpha", 1, "Amount of transparency to apply to the arrow.",
                                       this, SLOT( updateColorAndAlpha() ));
  alpha_property_->setMin( 0 );
  alpha_property_->setMax( 1 );

  shaft_length_property_ = new FloatProperty( "Shaft Length", 1, "Length of the arrow's shaft, in meters.",
                                              this, SLOT( updateArrowGeometry() ));

  // aleeper: default changed from 0.1 to match change in arrow.cpp
  shaft_radius_property_ = new FloatProperty( "Shaft Radius", 0.05, "Radius of the arrow's shaft, in meters.",
                                              this, SLOT( updateArrowGeometry() ));
  
  head_length_property_ = new FloatProperty( "Head Length", 0.3, "Length of the arrow's head, in meters.",
                                             this, SLOT( updateArrowGeometry() ));

  // aleeper: default changed from 0.2 to match change in arrow.cpp
  head_radius_property_ = new FloatProperty( "Head Radius", 0.1, "Radius of the arrow's head, in meters.",
                                             this, SLOT( updateArrowGeometry() ));

  axes_length_property_ = new FloatProperty( "Axes Length", 1, "Length of each axis, in meters.",
                                             this, SLOT( updateAxisGeometry() ));

  axes_radius_property_ = new FloatProperty( "Axes Radius", 0.1, "Radius of each axis, in meters.",
                                             this, SLOT( updateAxisGeometry() ));
}

void PoseDisplay::onInitialize()
{
  MFDClass::onInitialize();

  arrow_ = new rviz::Arrow( scene_manager_, scene_node_,
                            shaft_length_property_->getFloat(),
                            shaft_radius_property_->getFloat(),
                            head_length_property_->getFloat(),
                            head_radius_property_->getFloat() );
  // Arrow points in -Z direction, so rotate the orientation before display.
  // TODO: is it safe to change Arrow to point in +X direction?
  arrow_->setOrientation( Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y ));

  axes_ = new rviz::Axes( scene_manager_, scene_node_,
                          axes_length_property_->getFloat(),
                          axes_radius_property_->getFloat() );

  updateShapeChoice();
  updateColorAndAlpha();

  coll_handler_.reset( new PoseDisplaySelectionHandler( this, context_ ));
  coll_handler_->addTrackedObjects( arrow_->getSceneNode() );
  coll_handler_->addTrackedObjects( axes_->getSceneNode() );
}

PoseDisplay::~PoseDisplay()
{
  if ( initialized() )
  {
    delete arrow_;
    delete axes_;
  }
}

void PoseDisplay::onEnable()
{
  MFDClass::onEnable();
  updateShapeVisibility();
}

void PoseDisplay::updateColorAndAlpha()
{
  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  arrow_->setColor( color );

  context_->queueRender();
}

void PoseDisplay::updateArrowGeometry()
{
  arrow_->set( shaft_length_property_->getFloat(),
               shaft_radius_property_->getFloat(),
               head_length_property_->getFloat(),
               head_radius_property_->getFloat() );
  context_->queueRender();
}

void PoseDisplay::updateAxisGeometry()
{
  axes_->set( axes_length_property_->getFloat(),
              axes_radius_property_->getFloat() );
  context_->queueRender();
}

void PoseDisplay::updateShapeChoice()
{
  bool use_arrow = ( shape_property_->getOptionInt() == Arrow );

  color_property_->setHidden( !use_arrow );
  alpha_property_->setHidden( !use_arrow );
  shaft_length_property_->setHidden( !use_arrow );
  shaft_radius_property_->setHidden( !use_arrow );
  head_length_property_->setHidden( !use_arrow );
  head_radius_property_->setHidden( !use_arrow );

  axes_length_property_->setHidden( use_arrow );
  axes_radius_property_->setHidden( use_arrow );

  updateShapeVisibility();

  context_->queueRender();
}

void PoseDisplay::updateShapeVisibility()
{
  if( !pose_valid_ )
  {
    arrow_->getSceneNode()->setVisible( false );
    axes_->getSceneNode()->setVisible( false );
  }
  else
  {
    bool use_arrow = (shape_property_->getOptionInt() == Arrow);
    arrow_->getSceneNode()->setVisible( use_arrow );
    axes_->getSceneNode()->setVisible( !use_arrow );
  }
}

void PoseDisplay::processMessage( const geometry_msgs::PoseStamped::ConstPtr& message )
{
  if( !validateFloats( *message ))
  {
    setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  if( !validateQuaternions( *message ))
  {
    ROS_WARN_ONCE_NAMED( "quaternions", "Pose '%s' contains unnormalized quaternions. "
                          "This warning will only be output once but may be true for others; "
                          "enable DEBUG messages for ros.rviz.quaternions to see more details.",
                          qPrintable( getName() ) );
    ROS_DEBUG_NAMED( "quaternions", "Pose '%s' contains unnormalized quaternions.",
                     qPrintable( getName() ) );
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->transform( message->header, message->pose, position, orientation ))
  {
    ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
               qPrintable( getName() ), message->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  pose_valid_ = true;
  updateShapeVisibility();

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  coll_handler_->setMessage( message );

  context_->queueRender();
}

void PoseDisplay::reset()
{
  MFDClass::reset();
  pose_valid_ = false;
  updateShapeVisibility();
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::PoseDisplay, rviz::Display )
