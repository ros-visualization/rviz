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

#include <stdio.h>

#include <gtest/gtest.h>

#include <rviz/properties/property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/enum_property.h>

#include "mock_property_change_receiver.h"

using namespace rviz;

TEST( Property, name )
{
  Property p;
  p.setName( "chub" );
  EXPECT_EQ( "chub", p.getName().toStdString() );
}

TEST( Property, description )
{
  Property p;
  p.setDescription( "chub" );
  EXPECT_EQ( "chub", p.getDescription().toStdString() );
}

TEST( Property, value )
{
  Property p;
  p.setValue( 199 );
  EXPECT_EQ( 199, p.getValue().toInt() );
}

TEST( Property, set_value_events )
{
  Property p;
  p.setValue( 0 );

  MockPropertyChangeReceiver r( &p );
  p.connect( &p, SIGNAL( aboutToChange() ), &r, SLOT( aboutToChange() ));
  p.connect( &p, SIGNAL( changed() ), &r, SLOT( changed() ));

  p.setValue( 17 );
  EXPECT_EQ( " aboutToChange, v=0 changed, v=17", r.result().toStdString() );
}

TEST( Property, children )
{
  Property* display = new Property( "Test" );
  new Property( "Alpha", 0.5, "The amount of transparency to apply to the grid lines.", display );
  Property* beta = new Property( "Beta Band", 10, "The number of betas to apply to the grid lines.", display );
  new Property( "Gamma Topic", "chubby", "The topic on which to listen for Gamma messages.", display );
  Property* position = new Property( "Position", QVariant(), "Position of the chub.", display );
  new Property( "X", 1.1f, "X component of the position of the chub.", position );
  new Property( "Y", 0.717f, "Y component of the position of the chub.", position );

  // Sample usage inside the Display which owns the property.
  int b = beta->getValue().toInt();
  EXPECT_EQ( 10, b );

  // Sample usage outside the Display
  beta->setValue( 12 );
  EXPECT_EQ( 12, display->subProp( "Beta Band" )->getValue().toInt() );

  // Compound property
  float y = display->subProp( "Position" )->subProp( "Y" )->getValue().toFloat();
  EXPECT_EQ( 0.717f, y );

  // Accessing a missing property should not crash.
  printf( "Next line should say 'ERROR' but not crash.\n" );
  display->subProp( "Position" )->subProp( "Z" )->getValue().toFloat();
}

TEST( VectorProperty, default_value )
{
  VectorProperty* vp = new VectorProperty();
  Ogre::Vector3 vec = vp->getVector();
  EXPECT_EQ( 0, vec.x );
  EXPECT_EQ( 0, vec.y );
  EXPECT_EQ( 0, vec.z );
}

TEST( VectorProperty, set_and_get )
{
  VectorProperty* vp = new VectorProperty();
  Ogre::Vector3 vec( 1, 2, 3 );
  vp->setVector( vec );

  Ogre::Vector3 vec2 = vp->getVector();
  EXPECT_EQ( 1, vec2.x );
  EXPECT_EQ( 2, vec2.y );
  EXPECT_EQ( 3, vec2.z );
}

TEST( VectorProperty, set_string )
{
  VectorProperty* vp = new VectorProperty();
  vp->setValue( QString( "1;2;3" ));

  Ogre::Vector3 vec = vp->getVector();
  EXPECT_EQ( 1, vec.x );
  EXPECT_EQ( 2, vec.y );
  EXPECT_EQ( 3, vec.z );

  vp->setValue( QString( "chubby!" ));

  vec = vp->getVector();
  EXPECT_EQ( 1, vec.x );
  EXPECT_EQ( 2, vec.y );
  EXPECT_EQ( 3, vec.z );
}

TEST( VectorProperty, set_child )
{
  VectorProperty* vp = new VectorProperty();
  vp->subProp( "X" )->setValue( 0.9 );
  vp->subProp( "Y" )->setValue( 1.1 );
  vp->subProp( "Z" )->setValue( 1.3 );

  Ogre::Vector3 vec = vp->getVector();
  EXPECT_EQ( 0.9f, vec.x );
  EXPECT_EQ( 1.1f, vec.y );
  EXPECT_EQ( 1.3f, vec.z );
}

TEST( VectorProperty, get_child )
{
  VectorProperty* vp = new VectorProperty( "v", Ogre::Vector3( 1, 2, 3 ));
  EXPECT_EQ( 1, vp->subProp( "X" )->getValue().toFloat() );
  EXPECT_EQ( 2, vp->subProp( "Y" )->getValue().toFloat() );
  EXPECT_EQ( 3, vp->subProp( "Z" )->getValue().toFloat() );
}

TEST( VectorProperty, set_value_events )
{
  VectorProperty p;

  MockPropertyChangeReceiver r( &p );
  p.connect( &p, SIGNAL( aboutToChange() ), &r, SLOT( aboutToChange() ));
  p.connect( &p, SIGNAL( changed() ), &r, SLOT( changed() ));

  p.setVector( Ogre::Vector3( .1, .0001, 1000 ));
  EXPECT_EQ( " aboutToChange, v=0; 0; 0 changed, v=0.1; 0.0001; 1000", r.result().toStdString() );
  r.reset();

  p.subProp( "Z" )->setValue( 2.1 );
  EXPECT_EQ( " aboutToChange, v=0.1; 0.0001; 1000 changed, v=0.1; 0.0001; 2.1", r.result().toStdString() );
}

TEST( QuaternionProperty, default_value )
{
  QuaternionProperty* qp = new QuaternionProperty();
  Ogre::Quaternion quat = qp->getQuaternion();
  EXPECT_EQ( 0, quat.x );
  EXPECT_EQ( 0, quat.y );
  EXPECT_EQ( 0, quat.z );
  EXPECT_EQ( 1, quat.w );
}

TEST( QuaternionProperty, set_and_get )
{
  QuaternionProperty* qp = new QuaternionProperty();
  Ogre::Quaternion quat( 4, 1, 2, 3 );
  qp->setQuaternion( quat );

  Ogre::Quaternion quat2 = qp->getQuaternion();
  EXPECT_EQ( 1, quat2.x );
  EXPECT_EQ( 2, quat2.y );
  EXPECT_EQ( 3, quat2.z );
  EXPECT_EQ( 4, quat2.w );
}

TEST( QuaternionProperty, set_string )
{
  QuaternionProperty* qp = new QuaternionProperty();
  qp->setValue( QString( "1;2;3;4" ));

  Ogre::Quaternion quat = qp->getQuaternion();
  EXPECT_EQ( 1, quat.x );
  EXPECT_EQ( 2, quat.y );
  EXPECT_EQ( 3, quat.z );
  EXPECT_EQ( 4, quat.w );

  qp->setValue( QString( "chubby!" ));

  quat = qp->getQuaternion();
  EXPECT_EQ( 1, quat.x );
  EXPECT_EQ( 2, quat.y );
  EXPECT_EQ( 3, quat.z );
  EXPECT_EQ( 4, quat.w );
}

TEST( QuaternionProperty, set_child )
{
  QuaternionProperty* qp = new QuaternionProperty();
  qp->subProp( "X" )->setValue( 0.9 );
  qp->subProp( "Y" )->setValue( 1.1 );
  qp->subProp( "Z" )->setValue( 1.3 );
  qp->subProp( "W" )->setValue( 1.5 );

  Ogre::Quaternion quat = qp->getQuaternion();
  EXPECT_EQ( 0.9f, quat.x );
  EXPECT_EQ( 1.1f, quat.y );
  EXPECT_EQ( 1.3f, quat.z );
  EXPECT_EQ( 1.5f, quat.w );
}

TEST( QuaternionProperty, get_child )
{
  QuaternionProperty* qp = new QuaternionProperty( "v", Ogre::Quaternion( 4, 1, 2, 3 ));
  EXPECT_EQ( 1, qp->subProp( "X" )->getValue().toFloat() );
  EXPECT_EQ( 2, qp->subProp( "Y" )->getValue().toFloat() );
  EXPECT_EQ( 3, qp->subProp( "Z" )->getValue().toFloat() );
  EXPECT_EQ( 4, qp->subProp( "W" )->getValue().toFloat() );
}

TEST( QuaternionProperty, set_value_events )
{
  QuaternionProperty p;

  MockPropertyChangeReceiver r( &p );
  p.connect( &p, SIGNAL( aboutToChange() ), &r, SLOT( aboutToChange() ));
  p.connect( &p, SIGNAL( changed() ), &r, SLOT( changed() ));

  p.setQuaternion( Ogre::Quaternion( 1, .1, .0001, 1000 ));
  EXPECT_EQ( " aboutToChange, v=0; 0; 0; 1 changed, v=0.1; 0.0001; 1000; 1", r.result().toStdString() );
  r.reset();

  p.subProp( "Z" )->setValue( 2.1 );
  EXPECT_EQ( " aboutToChange, v=0.1; 0.0001; 1000; 1 changed, v=0.1; 0.0001; 2.1; 1", r.result().toStdString() );
}

TEST( ColorProperty, default_value )
{
  ColorProperty* qp = new ColorProperty();
  QColor color = qp->getColor();
  EXPECT_EQ( 0, color.red() );
  EXPECT_EQ( 0, color.green() );
  EXPECT_EQ( 0, color.blue() );
}

TEST( ColorProperty, set_and_get )
{
  ColorProperty* qp = new ColorProperty();
  QColor color( 1, 2, 3 );
  qp->setColor( color );

  QColor color2 = qp->getColor();
  EXPECT_EQ( 1, color2.red() );
  EXPECT_EQ( 2, color2.green() );
  EXPECT_EQ( 3, color2.blue() );
}

TEST( ColorProperty, set_string )
{
  ColorProperty* qp = new ColorProperty();
  qp->setValue( QString( "1;2;3" ));

  QColor color = qp->getColor();
  EXPECT_EQ( 1, color.red() );
  EXPECT_EQ( 2, color.green() );
  EXPECT_EQ( 3, color.blue() );

  qp->setValue( QString( "chubby!" ));

  color = qp->getColor();
  EXPECT_EQ( 1, color.red() );
  EXPECT_EQ( 2, color.green() );
  EXPECT_EQ( 3, color.blue() );
}

TEST( ColorProperty, set_string_limits )
{
  ColorProperty* qp = new ColorProperty();
  qp->setValue( QString( "-1;2000;3" ));

  QColor color = qp->getColor();
  EXPECT_EQ( 0, color.red() );
  EXPECT_EQ( 255, color.green() );
  EXPECT_EQ( 3, color.blue() );
}

TEST( ColorProperty, set_value_events )
{
  ColorProperty p;

  MockPropertyChangeReceiver r( &p );
  p.connect( &p, SIGNAL( aboutToChange() ), &r, SLOT( aboutToChange() ));
  p.connect( &p, SIGNAL( changed() ), &r, SLOT( changed() ));

  p.setColor( QColor( 1, 2, 3 ));
  EXPECT_EQ( " aboutToChange, v=0; 0; 0 changed, v=1; 2; 3", r.result().toStdString() );
}

TEST( EnumProperty, basic )
{
  EnumProperty p;

  EXPECT_EQ( 0, p.getOptionInt() );

  p.addOption( "chub", 10 );
  EXPECT_EQ( 0, p.getOptionInt() );

  p.addOption( "foo", 3 );
  p.addOption( "bar", 999 );

  p.setValue( "chub" );
  EXPECT_EQ( 10, p.getOptionInt() );

  p.clearOptions();
  EXPECT_EQ( 0, p.getOptionInt() );
}

int main( int argc, char **argv ) {
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
