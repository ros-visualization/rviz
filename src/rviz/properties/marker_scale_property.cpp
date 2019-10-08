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

#include <QStringList>

#include "rviz/properties/marker_scale_property.h"
#include <visualization_msgs/Marker.h>

namespace rviz
{

MarkerScaleProperty::MarkerScaleProperty( const QString& name,
                                          const Ogre::Vector3& default_value,
                                          const int32_t default_type,
                                          const QString& description,
                                          Property* parent,
                                          const char *changed_slot,
                                          QObject* receiver )
  : Property( name, QVariant(), description, parent, changed_slot, receiver )
  , scale_( default_value )
  , marker_type_ ( default_type )
  , ignore_child_updates_( false )
{
  x_ = new Property( "X", scale_.x, "", this );
  y_ = new Property( "Y", scale_.y, "", this );
  z_ = new Property( "Z", scale_.z, "", this );
  updateString();
  connect( x_, SIGNAL( aboutToChange() ), this, SLOT( emitAboutToChange() ));
  connect( y_, SIGNAL( aboutToChange() ), this, SLOT( emitAboutToChange() ));
  connect( z_, SIGNAL( aboutToChange() ), this, SLOT( emitAboutToChange() ));
  connect( x_, SIGNAL( changed() ), this, SLOT( updateFromChildren() ));
  connect( y_, SIGNAL( changed() ), this, SLOT( updateFromChildren() ));
  connect( z_, SIGNAL( changed() ), this, SLOT( updateFromChildren() ));
}

bool MarkerScaleProperty::setScale( const Ogre::Vector3& new_scale, int32_t new_marker_type )
{
  if( (new_scale != scale_) || (new_marker_type != marker_type_) ) {
    Q_EMIT aboutToChange();
    scale_ = new_scale;
    marker_type_ = new_marker_type;
    ignore_child_updates_ = true;
    x_->setValue( scale_.x );
    y_->setValue( scale_.y );
    z_->setValue( scale_.z );
    ignore_child_updates_ = false;
    updateString();
    Q_EMIT changed();
    return true;
  }
  return false;
}

bool MarkerScaleProperty::setValue( const QVariant& new_value )
{
  QStringList strings = new_value.toString().split( ';' );
  if( strings.size() >= 3 )
  {
    bool x_ok = true;
    float x = strings[ 0 ].toFloat( &x_ok );
    bool y_ok = true;
    float y = strings[ 1 ].toFloat( &y_ok );
    bool z_ok = true;
    float z = strings[ 2 ].toFloat( &z_ok );
    if( x_ok && y_ok && z_ok )
    {
      return setScale( Ogre::Vector3( x, y, z ), 0 );
    }
  }
  return false;
}

void MarkerScaleProperty::updateFromChildren()
{
  if( !ignore_child_updates_ )
  {
    scale_.x = x_->getValue().toFloat();
    scale_.y = y_->getValue().toFloat();
    scale_.z = z_->getValue().toFloat();
    updateString();
    Q_EMIT changed();
  }
}

void MarkerScaleProperty::emitAboutToChange()
{
  if( !ignore_child_updates_ )
  {
    Q_EMIT aboutToChange();
  }
}

void MarkerScaleProperty::updateString()
{
  switch (marker_type_) {
  case visualization_msgs::Marker::ARROW:
    x_->setName("Arrow Length");
    y_->setName("Arrow Width");
    z_->setName("Arrow Height");
    break;
  case visualization_msgs::Marker::CUBE:
  case visualization_msgs::Marker::CUBE_LIST:
  case visualization_msgs::Marker::TRIANGLE_LIST:
    x_->setName("Length");
    y_->setName("Width");
    z_->setName("Height");
    break;
  case visualization_msgs::Marker::SPHERE:
  case visualization_msgs::Marker::SPHERE_LIST:
    x_->setName("X Diameter");
    y_->setName("Y Diameter");
    z_->setName("Z Diameter");
    break;
  case visualization_msgs::Marker::CYLINDER:
    x_->setName("X Diameter");
    y_->setName("Y Diameter");
    z_->setName("Height");
    break;
  case visualization_msgs::Marker::LINE_STRIP:
  case visualization_msgs::Marker::LINE_LIST:
    x_->setName("Line Thickness");
    y_->setName("");
    z_->setName("");
    break;
  case visualization_msgs::Marker::POINTS:
    x_->setName("Point Width");
    y_->setName("Point Height");
    z_->setName("");
    break;
  case visualization_msgs::Marker::TEXT_VIEW_FACING:
    x_->setName("");
    y_->setName("");
    z_->setName("Text Height");
    break;
  case visualization_msgs::Marker::MESH_RESOURCE:
    x_->setName("X");
    y_->setName("Y");
    z_->setName("Z");
    break;
  }
  value_ = QString( "%1; %2; %3" )
    .arg( scale_.x, 0, 'g', 5 )
    .arg( scale_.y, 0, 'g', 5 )
    .arg( scale_.z, 0, 'g', 5 );
}

void MarkerScaleProperty::load( const Config& config )
{
  float x, y, z;
  if( config.mapGetFloat( "X", &x ) &&
      config.mapGetFloat( "Y", &y ) &&
      config.mapGetFloat( "Z", &z ))
  {
    setScale( Ogre::Vector3( x, y, z ), visualization_msgs::Marker::MESH_RESOURCE );
  }
}

void MarkerScaleProperty::save( Config config ) const
{
  // Saving the child values explicitly avoids having Property::save()
  // save the summary string version of the property.
  config.mapSetValue( "X", x_->getValue() );
  config.mapSetValue( "Y", y_->getValue() );
  config.mapSetValue( "Z", z_->getValue() );
}

void MarkerScaleProperty::setReadOnly( bool read_only )
{
  Property::setReadOnly( read_only );
  x_->setReadOnly( read_only );
  y_->setReadOnly( read_only );
  z_->setReadOnly( read_only );
}


} // end namespace rviz
