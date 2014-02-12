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

#include "axes.h"
#include "shape.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <sstream>

namespace rviz
{

Axes::Axes( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, float length, float radius )
    : Object( scene_manager )
{
  if ( !parent_node )
  {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();

  x_axis_ = new Shape( Shape::Cylinder, scene_manager_, scene_node_ );
  y_axis_ = new Shape( Shape::Cylinder, scene_manager_, scene_node_ );
  z_axis_ = new Shape( Shape::Cylinder, scene_manager_, scene_node_ );

  set( length, radius );
}

Axes::~Axes()
{
  delete x_axis_;
  delete y_axis_;
  delete z_axis_;

  scene_manager_->destroySceneNode( scene_node_->getName() );
}

void Axes::set( float length, float radius )
{
  x_axis_->setScale(Ogre::Vector3( radius, length, radius ));
  y_axis_->setScale(Ogre::Vector3( radius, length, radius ));
  z_axis_->setScale(Ogre::Vector3( radius, length, radius ));

  x_axis_->setPosition( Ogre::Vector3( length/2.0f, 0.0f, 0.0f ) );
  x_axis_->setOrientation( Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Z ) );
  y_axis_->setPosition( Ogre::Vector3( 0.0f, length/2.0f, 0.0f ) );
  z_axis_->setPosition( Ogre::Vector3( 0.0, 0.0f, length/2.0f ) );
  z_axis_->setOrientation( Ogre::Quaternion( Ogre::Degree( 90 ), Ogre::Vector3::UNIT_X ) );

  setToDefaultColors();
}

void Axes::setPosition( const Ogre::Vector3& position )
{
  scene_node_->setPosition( position );
}

void Axes::setOrientation( const Ogre::Quaternion& orientation )
{
  scene_node_->setOrientation( orientation );
}

void Axes::setScale( const Ogre::Vector3& scale )
{
  scene_node_->setScale( scale );
}

void Axes::setColor( float r, float g, float b, float a )
{
  // for now, do nothing
  /// \todo should anything be done here?
}

const Ogre::Vector3& Axes::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& Axes::getOrientation()
{
  return scene_node_->getOrientation();
}

void Axes::setUserData( const Ogre::Any& data )
{
  x_axis_->setUserData( data );
  y_axis_->setUserData( data );
  z_axis_->setUserData( data );
}

void Axes::setXColor(const Ogre::ColourValue& col)
{
  x_axis_->setColor(col.r, col.g, col.b, col.a);
}

void Axes::setYColor(const Ogre::ColourValue& col)
{
  y_axis_->setColor(col.r, col.g, col.b, col.a);
}

void Axes::setZColor(const Ogre::ColourValue& col)
{
  z_axis_->setColor(col.r, col.g, col.b, col.a);
}

void Axes::setToDefaultColors()
{
  x_axis_->setColor( 1.0f, 0.0f, 0.0f, 1.0f );
  y_axis_->setColor( 0.0f, 1.0f, 0.0f, 1.0f );
  z_axis_->setColor( 0.0f, 0.0f, 1.0f, 1.0f );
}

const Ogre::ColourValue Axes::default_x_color_( 1, 0, 0, 1 );
const Ogre::ColourValue Axes::default_y_color_( 0, 1, 0, 1 );
const Ogre::ColourValue Axes::default_z_color_( 0, 0, 1, 1 );

const Ogre::ColourValue& Axes::getDefaultXColor()
{
  return default_x_color_;
}

const Ogre::ColourValue& Axes::getDefaultYColor()
{
  return default_y_color_;
}

const Ogre::ColourValue& Axes::getDefaultZColor()
{
  return default_z_color_;
}

} // namespace rviz

