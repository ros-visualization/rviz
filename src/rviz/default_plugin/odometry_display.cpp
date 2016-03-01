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

#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/covariance_property.h"
#include "rviz/validate_floats.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "odometry_display.h"
#include "covariance_visual.h"

namespace rviz
{

OdometryDisplay::OdometryDisplay()
{

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

  shape_property_ = new EnumProperty( "Shape", "Arrow", "Shape to display the pose as.",
                                      this, SLOT( updateShapeChoice() ));
  shape_property_->addOption( "Arrow", ArrowShape );
  shape_property_->addOption( "Axes", AxesShape );

  color_property_ = new ColorProperty( "Color", QColor( 255, 25, 0 ),
                                       "Color of the arrows.",
                                       shape_property_, SLOT( updateColorAndAlpha() ), this);

  alpha_property_ = new FloatProperty( "Alpha", 1, "Amount of transparency to apply to the arrow.",
                                       shape_property_, SLOT( updateColorAndAlpha() ), this);
  alpha_property_->setMin( 0 );
  alpha_property_->setMax( 1 );

  shaft_length_property_ = new FloatProperty( "Shaft Length", 1, "Length of the each arrow's shaft, in meters.",
                                              shape_property_, SLOT( updateArrowsGeometry() ), this);

  // aleeper: default changed from 0.1 to match change in arrow.cpp
  shaft_radius_property_ = new FloatProperty( "Shaft Radius", 0.05, "Radius of the each arrow's shaft, in meters.",
                                              shape_property_, SLOT( updateArrowsGeometry() ), this);
  
  head_length_property_ = new FloatProperty( "Head Length", 0.3, "Length of the each arrow's head, in meters.",
                                             shape_property_, SLOT( updateArrowsGeometry() ), this);

  // aleeper: default changed from 0.2 to match change in arrow.cpp
  head_radius_property_ = new FloatProperty( "Head Radius", 0.1, "Radius of the each arrow's head, in meters.",
                                             shape_property_, SLOT( updateArrowsGeometry() ), this);

  axes_length_property_ = new FloatProperty( "Axes Length", 1, "Length of each axis, in meters.",
                                             shape_property_, SLOT( updateAxisGeometry() ), this);

  axes_radius_property_ = new FloatProperty( "Axes Radius", 0.1, "Radius of each axis, in meters.",
                                             shape_property_, SLOT( updateAxisGeometry() ), this);

  covariance_property_ = new CovarianceProperty( "Covariance", true, "Whether or not the covariances of the messages should be shown.",
                                             this, SLOT( updateCovarianceChoice() ));
 
  connect(covariance_property_, SIGNAL( childrenChanged() ), this, SLOT( updateCovarianceColorAndAlphaAndScale() ));

}

OdometryDisplay::~OdometryDisplay()
{
  if ( initialized() )
  {
    clear();
  }
}

void OdometryDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateShapeChoice();
}

void OdometryDisplay::onEnable()
{
  MFDClass::onEnable();
  updateShapeVisibility();
  updateCovarianceVisibility();
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

  D_Covariance::iterator it_cov = covariances_.begin();
  D_Covariance::iterator end_cov = covariances_.end();
  for ( ; it_cov != end_cov; ++it_cov )
  {
    delete *it_cov;
  }
  covariances_.clear();

  D_Axes::iterator it_axes = axes_.begin();
  D_Axes::iterator end_axes = axes_.end();
  for ( ; it_axes != end_axes; ++it_axes )
  {
    delete *it_axes;
  }
  axes_.clear();

  if( last_used_message_ )
  {
    last_used_message_.reset();
  }
}

void OdometryDisplay::updateColorAndAlpha()
{
  QColor color = color_property_->getColor();
  float red   = color.redF();
  float green = color.greenF();
  float blue  = color.blueF();
  float alpha = alpha_property_->getFloat();

  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for( ; it != end; ++it )
  {
    Arrow* arrow = *it;
    arrow->setColor( red, green, blue, alpha );
  }
  context_->queueRender();
}

void OdometryDisplay::updateArrowsGeometry()
{
  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    updateGeometry(*it);
  }
  context_->queueRender();
}

void OdometryDisplay::updateAxisGeometry()
{
  D_Axes::iterator it = axes_.begin();
  D_Axes::iterator end = axes_.end();
  for ( ; it != end; ++it )
  {
    updateGeometry(*it);
  }
  context_->queueRender();
}

void OdometryDisplay::updateGeometry( Axes* axes )
{
    axes->set( axes_length_property_->getFloat(),
              axes_radius_property_->getFloat() );
}

void OdometryDisplay::updateGeometry( Arrow* arrow )
{
    arrow->set( shaft_length_property_->getFloat(),
                shaft_radius_property_->getFloat(),
                head_length_property_->getFloat(),
                head_radius_property_->getFloat() );
}

void OdometryDisplay::updateShapeChoice()
{
  bool use_arrow = ( shape_property_->getOptionInt() == ArrowShape );

  color_property_->setHidden( !use_arrow );
  alpha_property_->setHidden( !use_arrow );
  shaft_length_property_->setHidden( !use_arrow );
  shaft_radius_property_->setHidden( !use_arrow );
  head_length_property_->setHidden( !use_arrow );
  head_radius_property_->setHidden( !use_arrow );

  axes_length_property_->setHidden( use_arrow );
  axes_radius_property_->setHidden( use_arrow );

  updateShapeVisibility();
  // covariances are children of axis, thus we need to update their 
  // visibilities as well in case their parents turns invisible
  updateCovarianceVisibility();

  context_->queueRender();
}

void OdometryDisplay::updateShapeVisibility()
{
  bool use_arrow = (shape_property_->getOptionInt() == ArrowShape);

  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    (*it)->getSceneNode()->setVisible( use_arrow );
  }

  D_Axes::iterator it_axes = axes_.begin();
  D_Axes::iterator end_axes = axes_.end();
  for ( ; it_axes != end_axes; ++it_axes )
  {
    (*it_axes)->getSceneNode()->setVisible( !use_arrow );
  }
}

void OdometryDisplay::updateCovarianceChoice()
{
  updateCovarianceVisibility();
  context_->queueRender();
}

void OdometryDisplay::updateCovarianceVisibility()
{
  bool show_covariance = covariance_property_->getBool();

  D_Covariance::iterator it_cov = covariances_.begin();
  D_Covariance::iterator end_cov = covariances_.end();
  for ( ; it_cov != end_cov; ++it_cov )
  {
    CovarianceVisual* cov = *it_cov;
    cov->setVisible( show_covariance );
  }
}

void OdometryDisplay::updateCovarianceColorAndAlphaAndScale()
{
  QColor pos_color = covariance_property_->getPositionColor();
  float pos_alpha = covariance_property_->getPositionAlpha();
  float pos_scale = covariance_property_->getPositionScale();

  QColor ori_color = covariance_property_->getOrientationColor();
  float ori_alpha = covariance_property_->getOrientationAlpha();
  float ori_scale = covariance_property_->getOrientationScale();

  D_Covariance::iterator it_cov = covariances_.begin();
  D_Covariance::iterator end_cov = covariances_.end();
  for ( ; it_cov != end_cov; ++it_cov )
  {
    CovarianceVisual* cov = *it_cov;

    cov->setPositionColor( pos_color.redF(), pos_color.greenF(), pos_color.blueF(), pos_alpha );
    cov->setPositionScale( pos_scale );

    cov->setOrientationColor( ori_color.redF(), ori_color.greenF(), ori_color.blueF(), ori_alpha );
    cov->setOrientationScale( ori_scale );
  }

  context_->queueRender();
}

bool validateFloats(const nav_msgs::Odometry& msg)
{
  bool valid = true;
  valid = valid && validateFloats( msg.pose.pose );
  valid = valid && validateFloats( msg.pose.covariance );
  valid = valid && validateFloats( msg.twist.twist );
  // valid = valid && validateFloats( msg.twist.covariance )
  return valid;
}

void OdometryDisplay::processMessage( const nav_msgs::Odometry::ConstPtr& message )
{
   if( !validateFloats( *message ))
  {
    setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

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

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->transform( message->header, message->pose.pose, position, orientation ))
  {
    ROS_ERROR( "Error transforming odometry '%s' from frame '%s' to frame '%s'",
               qPrintable( getName() ), message->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  Ogre::Vector3 frame_position;
  Ogre::Quaternion frame_orientation; 
  if( !context_->getFrameManager()->getTransform( message->header, frame_position, frame_orientation ))
  {
    ROS_ERROR( "Error recovering the transform from frame '%s' to frame '%s'",
               message->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  // If we arrive here, we're good. Continue...

  // Create a scene node, and attach the arrow and the covariance to it
  Axes* axes = new Axes( scene_manager_, scene_node_,
                         axes_length_property_->getFloat(),
                         axes_radius_property_->getFloat() );
  Arrow* arrow = new Arrow( scene_manager_, scene_node_, 
                            shaft_length_property_->getFloat(),
                            shaft_radius_property_->getFloat(),
                            head_length_property_->getFloat(),
                            head_radius_property_->getFloat() );
  // The axis will be the parent of the covariance
  CovarianceVisual* cov = new CovarianceVisual( scene_manager_, scene_node_ );

  // Position the axes
  axes->setPosition( position );
  axes->setOrientation( orientation );

  // Position the arrow. Remember the arrow points in -Z direction, so rotate the orientation before display.
  arrow->setPosition( position );

  // Arrow points in -Z direction, so rotate the orientation before display.
  arrow->setOrientation( orientation * Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y ));

  // Position the frame where the covariance is attached covariance
  cov->setFramePosition( frame_position );
  cov->setFrameOrientation( frame_orientation );

  // Set up arrow color
  QColor color = color_property_->getColor();
  float alpha = alpha_property_->getFloat();
  arrow->setColor( color.redF(), color.greenF(), color.blueF(), alpha);

  // Set up covariance color and scales
  color = covariance_property_->getPositionColor();
  alpha = covariance_property_->getPositionAlpha();
  cov->setPositionColor(color.redF(), color.greenF(), color.blueF(), covariance_property_->getPositionAlpha());

  color = covariance_property_->getOrientationColor();
  alpha = covariance_property_->getOrientationAlpha();
  cov->setOrientationColor(color.redF(), color.greenF(), color.blueF(), alpha);

  cov->setPositionScale( covariance_property_->getPositionScale() );
  cov->setOrientationScale( covariance_property_->getOrientationScale() );

  // Set up the covariance based on the message
  cov->setCovariance(message->pose);

  // Show/Hide things based on current properties
  bool use_arrow = (shape_property_->getOptionInt() == ArrowShape);
  arrow->getSceneNode()->setVisible( use_arrow );
  axes->getSceneNode()->setVisible( !use_arrow );
  cov->setVisible( covariance_property_->getBool() );

  // store everything
  axes_.push_back( axes );
  arrows_.push_back( arrow );
  covariances_.push_back( cov );

  last_used_message_ = message;
  context_->queueRender();
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

      delete covariances_.front();
      covariances_.pop_front();

      delete axes_.front();
      axes_.pop_front();
    }
  }

  assert(arrows_.size() == covariances_.size());
  assert(covariances_.size() == axes_.size());

}

void OdometryDisplay::reset()
{
  MFDClass::reset();
  clear();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::OdometryDisplay, rviz::Display )
