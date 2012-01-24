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

#include "axes_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"

#include "rviz/ogre_helpers/axes.h"

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

AxesDisplay::AxesDisplay()
  : Display()
  , length_( 1.0 )
  , radius_( 0.1 )
  , axes_( 0 )
{
}

AxesDisplay::~AxesDisplay()
{
  delete axes_;
}

void AxesDisplay::onInitialize()
{
  axes_ = new Axes( scene_manager_, 0, length_, radius_ );

  axes_->getSceneNode()->setVisible( isEnabled() );

  setFrame(FIXED_FRAME_STRING);
}

void AxesDisplay::onEnable()
{
  axes_->getSceneNode()->setVisible( true );
}

void AxesDisplay::onDisable()
{
  axes_->getSceneNode()->setVisible( false );
}

void AxesDisplay::create()
{
  axes_->set( length_, radius_ );

  causeRender();
}

void AxesDisplay::set( float length, float radius )
{
  length_ = length;
  radius_ = radius;

  create();

  propertyChanged(length_property_);
  propertyChanged(radius_property_);
}

void AxesDisplay::setFrame(const std::string& frame)
{
  frame_ = frame;
  propertyChanged(frame_property_);
}

void AxesDisplay::setLength( float length )
{
  set( length, radius_ );
}

void AxesDisplay::setRadius( float radius )
{
  set( length_, radius );
}

void AxesDisplay::update(float dt, float ros_dt)
{
  std::string frame = frame_;
  if (frame == FIXED_FRAME_STRING)
  {
    frame = fixed_frame_;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (vis_manager_->getFrameManager()->getTransform(frame, ros::Time(), position, orientation))
  {
    axes_->setPosition(position);
    axes_->setOrientation(orientation);
    setStatus(status_levels::Ok, "Transform", "Transform OK");
  }
  else
  {
    std::string error;
    if (vis_manager_->getFrameManager()->transformHasProblems(frame, ros::Time(), error))
    {
      setStatus(status_levels::Error, "Transform", error);
    }
    else
    {
      std::stringstream ss;
      ss << "Could not transform from [" << frame << "] to Fixed Frame [" << fixed_frame_ << "] for an unknown reason";
      setStatus(status_levels::Error, "Transform", ss.str());
    }
  }
}

void AxesDisplay::createProperties()
{
  frame_property_ = property_manager_->createProperty<TFFrameProperty>("Reference Frame", property_prefix_, boost::bind(&AxesDisplay::getFrame, this),
                                                                       boost::bind(&AxesDisplay::setFrame, this, _1), parent_category_, this);
  setPropertyHelpText(frame_property_, "The TF frame these axes will use for their origin.");
  length_property_ = property_manager_->createProperty<FloatProperty>( "Length", property_prefix_, boost::bind( &AxesDisplay::getLength, this ),
                                                                     boost::bind( &AxesDisplay::setLength, this, _1 ), parent_category_, this );
  FloatPropertyPtr float_prop = length_property_.lock();
  float_prop->setMin( 0.0001 );
  setPropertyHelpText(length_property_, "Length of each axis, in meters.");

  radius_property_ = property_manager_->createProperty<FloatProperty>( "Radius", property_prefix_, boost::bind( &AxesDisplay::getRadius, this ),
                                                                       boost::bind( &AxesDisplay::setRadius, this, _1 ), parent_category_, this );
  float_prop = radius_property_.lock();
  float_prop->setMin( 0.0001 );
  setPropertyHelpText(radius_property_, "Width of each axis, in meters.");
}

} // namespace rviz
