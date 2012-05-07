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

#include "rviz/properties/parse_color.h"

namespace rviz
{

static int limit( int i )
{
  if( i < 0 )
    return 0;
  if( i > 255 )
    return 255;
  return i;
}

QColor parseColor( const QString& color_string )
{
  if( color_string.indexOf( ';' ) != -1 )
  {
    QStringList strings = color_string.split( ';' );
    if( strings.size() >= 3 )
    {
      bool r_ok = true;
      int r = strings[ 0 ].toInt( &r_ok );
      bool g_ok = true;
      int g = strings[ 1 ].toInt( &g_ok );
      bool b_ok = true;
      int b = strings[ 2 ].toInt( &b_ok );
      if( r_ok && g_ok && b_ok )
      {
        return QColor( limit( r ), limit( g ), limit( b ));
      }
    }
    return QColor();
  }

  QColor new_color;
  if( QColor::colorNames().contains( color_string, Qt::CaseInsensitive ) ||
      (color_string.size() > 0 && color_string[ 0 ] == '#' ))
  {
    new_color.setNamedColor( color_string.toLower() );
  }
  return new_color;
}

QString printColor( const QColor& color )
{
  return QString( "%1; %2; %3" )
    .arg( color.red() )
    .arg( color.green() )
    .arg( color.blue() );
}

QColor ogreToQt( const Ogre::ColourValue& c )
{
  return QColor::fromRgbF( c.r, c.g, c.b, c.a );
}

Ogre::ColourValue qtToOgre( const QColor& c )
{
  return Ogre::ColourValue( c.redF(), c.greenF(), c.blueF(), c.alphaF() );
}

} // end namespace rviz
