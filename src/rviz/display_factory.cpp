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

#include "rviz/display_group.h"

#include "rviz/display_factory.h"

namespace rviz
{

#define RVIZ_DISPLAY_GROUP_STRING "rviz/DisplayGroup"

DisplayFactory::DisplayFactory()
  : PluginlibFactory<Display>( "rviz", "rviz::Display" )
{}

QStringList DisplayFactory::getDeclaredClassIds()
{
  QStringList ids = PluginlibFactory<Display>::getDeclaredClassIds();
  ids.push_back( RVIZ_DISPLAY_GROUP_STRING );
  return ids;
}

QString DisplayFactory::getClassDescription( const QString& class_id ) const
{
  if( class_id == RVIZ_DISPLAY_GROUP_STRING )
  {
    return "A container for Displays.";
  }
  return PluginlibFactory<Display>::getClassDescription( class_id );
}

QString DisplayFactory::getClassName( const QString& class_id ) const
{
  if( class_id == RVIZ_DISPLAY_GROUP_STRING )
  {
    return "Group";
  }
  return PluginlibFactory<Display>::getClassName( class_id );
}

QString DisplayFactory::getClassPackage( const QString& class_id ) const
{
  if( class_id == RVIZ_DISPLAY_GROUP_STRING )
  {
    return "rviz";
  }
  return PluginlibFactory<Display>::getClassPackage( class_id );
}

Display* DisplayFactory::makeRaw( const QString& class_id, QString* error_return )
{
  Display* display;

  if( class_id == RVIZ_DISPLAY_GROUP_STRING )
  {
    display = new DisplayGroup();
  }
  else
  {
    display = PluginlibFactory<Display>::makeRaw( class_id, error_return );
  }
  if ( display )
  {
    display->setIcon( getIcon(class_id) );
  }
  return display;
}

} // end namespace rviz
