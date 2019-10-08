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

#include "rviz/properties/marker_color_property.h"

namespace rviz
{

MarkerColorProperty::MarkerColorProperty( const QString& name,
                                          const std_msgs::ColorRGBA& default_value,
                                          const QString& description,
                                          Property* parent,
                                          const char *changed_slot,
                                          QObject* receiver )
  : Property( name, QVariant(), description, parent, changed_slot, receiver )
  , color_( default_value )
  , ignore_child_updates_( false )
{
  a_ = new Property( "A", color_.a, "", this );
  r_ = new Property( "R", color_.r, "", this );
  g_ = new Property( "G", color_.g, "", this );
  b_ = new Property( "B", color_.b, "", this );
  updateString();
  connect( a_, SIGNAL( aboutToChange() ), this, SLOT( emitAboutToChange() ));
  connect( r_, SIGNAL( aboutToChange() ), this, SLOT( emitAboutToChange() ));
  connect( g_, SIGNAL( aboutToChange() ), this, SLOT( emitAboutToChange() ));
  connect( b_, SIGNAL( aboutToChange() ), this, SLOT( emitAboutToChange() ));
  connect( a_, SIGNAL( changed() ), this, SLOT( updateFromChildren() ));
  connect( r_, SIGNAL( changed() ), this, SLOT( updateFromChildren() ));
  connect( g_, SIGNAL( changed() ), this, SLOT( updateFromChildren() ));
  connect( b_, SIGNAL( changed() ), this, SLOT( updateFromChildren() ));
}

bool MarkerColorProperty::setColor( const std_msgs::ColorRGBA& new_color )
{
  return setColor(new_color.a, new_color.r, new_color.g, new_color.b);
}

bool MarkerColorProperty::setColor( float a, float r, float g, float b )
{
  if(    (a != color_.a) || (r != color_.r)
      || (g != color_.g) || (b != color_.b) ) {
    Q_EMIT aboutToChange();
    color_.a = a;
    color_.r = r;
    color_.g = g;
    color_.b = b;
    ignore_child_updates_ = true;
    a_->setValue( color_.a );
    r_->setValue( color_.r );
    g_->setValue( color_.g );
    b_->setValue( color_.b );
    ignore_child_updates_ = false;
    updateString();
    Q_EMIT changed();
    return true;
  }
  return false;
}

bool MarkerColorProperty::setValue( const QVariant& new_value )
{
  QStringList strings = new_value.toString().split( ';' );
  if( strings.size() >= 4 )
  {
    bool a_ok = true;
    float a = strings[ 0 ].toFloat( &a_ok );
    bool r_ok = true;
    float r = strings[ 1 ].toFloat( &r_ok );
    bool g_ok = true;
    float g = strings[ 2 ].toFloat( &g_ok );
    bool b_ok = true;
    float b = strings[ 3 ].toFloat( &b_ok );
    if( a_ok && r_ok && g_ok && b_ok )
    {
      return setColor( a, r, g, b );
    }
  }
  return false;
}

void MarkerColorProperty::updateFromChildren()
{
  if( !ignore_child_updates_ )
  {
    color_.a = a_->getValue().toFloat();
    color_.r = r_->getValue().toFloat();
    color_.g = g_->getValue().toFloat();
    color_.b = b_->getValue().toFloat();
    updateString();
    Q_EMIT changed();
  }
}

void MarkerColorProperty::emitAboutToChange()
{
  if( !ignore_child_updates_ )
  {
    Q_EMIT aboutToChange();
  }
}

void MarkerColorProperty::updateString()
{
  value_ = QString( "%1; %2; %3; %4" )
    .arg( color_.a, 0, 'g', 5 )
    .arg( color_.r, 0, 'g', 5 )
    .arg( color_.g, 0, 'g', 5 )
    .arg( color_.b, 0, 'g', 5 );
}

void MarkerColorProperty::load( const Config& config )
{
  float a, r, g, b;
  if( config.mapGetFloat( "A", &a ) &&
      config.mapGetFloat( "R", &r ) &&
      config.mapGetFloat( "G", &g ) &&
      config.mapGetFloat( "B", &b ))
  {
    setColor( a, r, g, b );
  }
}

void MarkerColorProperty::save( Config config ) const
{
  // Saving the child values explicitly avoids having Property::save()
  // save the summary string version of the property.
  config.mapSetValue( "A", a_->getValue() );
  config.mapSetValue( "R", r_->getValue() );
  config.mapSetValue( "G", g_->getValue() );
  config.mapSetValue( "B", b_->getValue() );
}

void MarkerColorProperty::setReadOnly( bool read_only )
{
  Property::setReadOnly( read_only );
  a_->setReadOnly( read_only );
  r_->setReadOnly( read_only );
  g_->setReadOnly( read_only );
  b_->setReadOnly( read_only );
}


} // end namespace rviz
